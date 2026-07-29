# Code generation and runtime for the constant-velocity friction experiment.
#
# `FurutaFriction` (dyad/friction.dyad) is a purely discrete system that contains the whole
# experiment: `HardwareMeasurement` reads the encoders, a PI loop drives the arm around the
# `VelocityStaircase` reference, `HardwareCommand` writes the motor, and `DataLogger` writes
# a row per tick. `stkcompile` turns it into a synchronous node whose only argument is a
# clock tick — so what is left out here is timing, and nothing else. In particular there is
# no logging code in the loop below: the program writes the file.
#
# The generated node has the runtime signature
#     (row, w_ref, shoulder_angle, shoulder_velocity, u_applied) = step(tick, gains, auto)
# where `gains` carries the two velocity-loop knobs `K` and `Ti` (runtime-settable, so the
# loop can be retuned without recompiling) and `auto` the remaining model parameters.
# `row` is the logger's row count, returned so the driver can check that the program wrote
# one row per tick.

export generate_friction_controller, FrictionController, run_friction_experiment,
       friction_sweep_duration

"""
    generate_friction_controller(; Ts=0.005, log_file=FRICTION_LOG_FILE, overrides...)

Compile the friction experiment to a SynchJulia node: build `FurutaFriction` on a
`PeriodicClock` at sample time `Ts`, with its `DataLogger` writing to `log_file`, and
`stkcompile` it.

Returns `(; topmod, Kdef, Tidef, log_file)`. The node argument order is
`(tick::Bool, gains::TuningGains, auto::AutoPars)` and the outputs are
`(row, w_ref, shoulder_angle, shoulder_velocity, u_applied)`.

`log_file` goes to the model, not just to the driver: it is the `DataLogger`'s structural
`filename` parameter, which is how [`FrictionController`](@ref) knows what to open. Pass
model parameters as `overrides` using Dyad's `__`-separated paths, e.g. `w_max = 20.0` for
the model's own parameters or `velocity_pi__wp = 1.0` for one of a sub-component's.

Note that `Ni`, the PI loop's anti-windup gain, has a default expression in `Ti` and so is
resolved once here rather than tracking a later runtime change to `Ti` — retuning `Ti` by a
large factor is worth a recompile.
"""
function generate_friction_controller(; Ts = 0.005, log_file = FRICTION_LOG_FILE,
                                        overrides...)
    # Both C libraries the program calls into have to exist before the `:c` backend links
    # them (the Julia backend only needs them at call time).
    ensure_qube_hw()
    ensure_qube_log()
    sys = FurutaFriction(; name = :friction, Ts, log_file, overrides...)
    # `sys` is the root, so its own name is not part of the flattened symbol names; reach
    # for symbols through the un-namespaced view, as `generate_swingup_controller` does.
    nsys = ModelingToolkit.toggle_namespacing(sys, false)

    # The velocity loop's own parameters, not lifted copies: a `ParametersStruct` field must
    # be an unbound parameter, which is why these live on `velocity_pi` (same reason
    # `TuningGains` in codegen.jl reaches for `lqrstabilizer.L` rather than a wrapper's).
    Ksym = ModelingToolkit.unwrap(nsys.velocity_pi.K)
    Tisym = ModelingToolkit.unwrap(nsys.velocity_pi.Ti)
    SymT = ModelingToolkit.SymbolicT
    dv = Dict{SymT, SymT}(ModelingToolkit.default_values(ModelingToolkit.expand_connections(sys)))
    ModelingToolkit.evaluate_varmap!(dv, [Ksym, Tisym])
    Kdef = Float64(Symbolics.value(dv[Ksym]))
    Tidef = Float64(Symbolics.value(dv[Tisym]))

    inputs = [
        InputClock(ModelingToolkit.Clock(Ts)),
        ParametersStruct(; arg_name = :gains, struct_name = :TuningGains,
                           parameters = OrderedDict{Any, Symbol}(Ksym => :K, Tisym => :Ti),
                           generated = false),
        ParametersStruct(; arg_name = :auto, struct_name = :AutoPars),
    ]
    # `row` first so the row count is the cheapest thing to check. The rest is what the
    # experiment is about, in the same order as the log's columns.
    outputs = [
        ClockedOutput(nsys.logger.row),
        ClockedOutput(nsys.reference.w_ref),
        ClockedOutput(nsys.measurement.shoulder_angle),
        ClockedOutput(nsys.velocityestimator.vel),
        ClockedOutput(nsys.command.u_applied),
    ]
    @info "Running stkcompile"
    topmod = SynchToolkit.stkcompile(sys; inputs, outputs)
    return (; topmod, Kdef, Tidef, log_file = String(log_file))
end

"""
    FrictionController(; Ts=0.005, backend=:julia, log_file=FRICTION_LOG_FILE, K=nothing,
                         Ti=nothing, overrides...)

A ready-to-call runtime wrapper around the generated friction experiment. Compiles the
program, builds a `SynchExecutable` on `backend` (`:julia` or `:c`), and populates the
parameter structs.

Advance one step with `out = controller()`, which reads the encoders, updates the velocity
reference and the PI loop, writes the motor voltage and appends a row to the log, returning
`(; row, w_ref, shoulder, velocity, u)`. Open the device with [`open_hardware!`](@ref) and
the log with [`open_log!`](@ref) first — or just call [`run_friction_experiment`](@ref),
which does both.

`K` and `Ti` override the velocity-loop gains at runtime; both default to the model's.
"""
struct FrictionController{E, G, A}
    exe::E
    gains::G
    auto::A
    log_file::String
    Ts::Float64
end

function FrictionController(; Ts = 0.005, backend::Symbol = :julia,
                             log_file = FRICTION_LOG_FILE, K = nothing, Ti = nothing,
                             kwargs...)
    gen = generate_friction_controller(; Ts, log_file, kwargs...)
    return _make_friction_runtime(gen; backend, K, Ti, Ts)
end

# Same single-world-age-crossing contract as `_instantiate` in codegen.jl: `stkcompile`
# evaluates the runtime module in a newer world than this frame, so every module member
# read and constructor call happens inside one `@invokelatest`.
function _make_friction_runtime(gen; backend::Symbol = :julia, K = nothing, Ti = nothing,
                                Ts = 0.005)
    Kv = float(something(K, gen.Kdef))
    Tiv = float(something(Ti, gen.Tidef))
    function consume(m)
        gains = m.TuningGains(; K = Kv, Ti = Tiv)
        auto = m.AutoPars(gains)
        return (; gains, auto, exe = m.executable(backend))
    end
    r = Base.@invokelatest consume(gen.topmod)
    return FrictionController(r.exe, r.gains, r.auto, gen.log_file, Float64(Ts))
end

"""
    (c::FrictionController)(; tick=true) -> (; row, w_ref, shoulder, velocity, u)

Advance the experiment one step. With `tick=false` the clock does not fire, so no hardware
is touched, no row is logged and the returned values are meaningless.
"""
function (c::FrictionController)(; tick::Bool = true)
    out = SynchJulia.step!(c.exe, tick, c.gains, c.auto)
    v = values(out)
    return (; row = v[1], w_ref = v[2], shoulder = v[3], velocity = v[4], u = v[5])
end

# Resets the reference schedule, the PI state and the velocity estimator; the device and
# the log stay as they are.
function SynchToolkit.reset!(c::FrictionController)
    SynchToolkit.reset!(c.exe)
    reset_hardware_counters!()
    return c
end

"""
    friction_sweep_duration(; n_levels=6, t_step=2.0) -> Float64

Seconds one full cycle of the `VelocityStaircase` reference takes: `n_levels` speeds in
each direction, `t_step` seconds each. This is the natural run length for the experiment —
shorter truncates the sweep, longer just repeats it.
"""
friction_sweep_duration(; n_levels = 6, t_step = 2.0) = 2 * float(n_levels) * float(t_step)

"""
    run_friction_experiment(ctrl; Tf, arm_deg=0.0, card_options="deadband_compensation=0.0")

Run `ctrl` against the physical QUBE for `Tf` seconds and return `(; rows, ticks, log_file,
timing)`.

This opens the device and the log, ticks the program every `Ts`, and closes both. It does
no logging of its own — the program writes the file — so the loop body is a single call,
and `rows` coming back equal to `ticks` is the check that the program wrote one row per
tick.

Deadband compensation is off by default: it offsets the command in the direction it points,
which is indistinguishable from the Coulomb friction this experiment measures. Pass
`card_options = nothing` to leave `qube_hw.c`'s own default in force instead.

`timing` is `(; median_dt, max_dt)` of the achieved period, for confirming the loop kept up.
"""
function run_friction_experiment(ctrl::FrictionController; Tf = friction_sweep_duration(),
                                 arm_deg = 0.0,
                                 card_options = "deadband_compensation=0.0")
    Ts = ctrl.Ts
    N = round(Int, Tf / Ts)
    tstamp = Vector{Float64}(undef, N)     # preallocated: GC is disabled below
    n = 0
    SynchToolkit.reset!(ctrl)
    open_log!(ctrl.log_file; header = FRICTION_LOG_HEADER, ncols = FRICTION_LOG_NCOLS)
    open_hardware!(:hil; arm_deg, card_options)
    try
        GC.gc()
        GC.enable(false)
        t0 = time()
        t_next = t0 + Ts
        for i in 1:N
            ctrl()
            tstamp[i] = time() - t0
            n = i
            # Absolute schedule rather than "sleep Ts": a step that overruns is absorbed
            # by the next period instead of shifting every period after it. `systemsleep`
            # because `sleep`'s resolution is of the same order as Ts itself.
            dt = t_next - time()
            dt > 0 && Libc.systemsleep(dt)
            t_next += Ts
        end
    catch e
        @error "Terminating friction experiment" e
    finally
        # The program cannot unwind a write it has already made, so make sure the motor
        # ends up at zero and the log is flushed whatever happened.
        close_hardware!()
        close_log!()
        GC.enable(true)
    end
    st = log_state()
    st.error && @warn "the log was closed early by a write error" file = st.filename
    dts = n > 1 ? diff(view(tstamp, 1:n)) : Float64[]
    return (; rows = st.rows, ticks = n, log_file = ctrl.log_file,
              timing = (; median_dt = _median(dts), max_dt = isempty(dts) ? NaN : maximum(dts)))
end

# Statistics is not a dependency of this package and one median does not justify making it
# one; the achieved periods are a short vector, so sorting a copy costs nothing.
function _median(v)
    isempty(v) && return NaN
    s = sort(collect(v))
    n = length(s)
    return isodd(n) ? s[(n + 1) ÷ 2] : (s[n ÷ 2] + s[n ÷ 2 + 1]) / 2
end
