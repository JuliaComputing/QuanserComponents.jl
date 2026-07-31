# The constant-velocity friction experiment as a synchronous program: what is specific to it.
#
# `FurutaFriction` (dyad/friction.dyad) is a purely discrete system that contains the whole
# experiment: `HardwareMeasurement` reads the encoders, a PI loop drives the arm around the
# `VelocityStaircase` reference, `HardwareCommand` writes the motor, and `DataLogger` writes a
# row per tick. `stkcompile` turns it into a synchronous node whose only argument is a clock
# tick -- so what is left out here is timing, and nothing else. In particular there is no
# logging code in the loop: the program writes the file.
#
# The generated node has the runtime signature
#     (row, w_ref, shoulder_angle, shoulder_velocity, u_applied) = step(tick, gains, auto)
# where `gains` carries the two velocity-loop knobs `K` and `Ti` (runtime-settable, so the
# loop can be retuned without recompiling) and `auto` the remaining model parameters. `row` is
# the logger's row count, returned so the driver can check that the program wrote one row per
# tick.
#
# Compiling, the runtime and the timing loop are shared with the swing-up controller and live
# in program.jl; getting either program onto hardware is harness.jl's job.

export generate_friction_controller, FrictionController, run_friction_experiment,
       friction_sweep_duration, friction_log

"The log `FurutaFriction` writes, in `FRICTION_LOG_COLUMNS` order. `file` defaults to `FRICTION_LOG_FILE`."
friction_log(file = FRICTION_LOG_FILE) = ProgramLog(file, FRICTION_LOG_COLUMNS)

# `FurutaFriction`'s own `K`/`Ti`, not `velocity_pi`'s: the loop's are bound to these by
# `final K = K`, and a `ParametersStruct` field must be an *unbound* parameter. `velocity_pi.K`
# then has `K` as its default expression, which `AutoPars(gains)` resolves from the struct --
# the same mechanism `Ni` relies on.
const FRICTION_TUNABLES = OrderedDict{Any, Symbol}(
    (nsys -> nsys.K) => :K,
    (nsys -> nsys.Ti) => :Ti,
)

# `row` first so the row count is the cheapest thing to check. The rest is what the experiment
# is about, in the same order as the log's columns.
_friction_outputs(nsys) = [nsys.logger.row, nsys.reference.w_ref,
                           nsys.measurement.shoulder_angle, nsys.velocityestimator.vel,
                           nsys.command.u_applied]
const FRICTION_OUTPUT_NAMES = (:row, :w_ref, :shoulder, :velocity, :u)

# The integrator is forward-Euler: `I += (Ts/Ti) * e` each tick. `Ti` below a few sample times
# therefore moves the integrator by more than the error itself every tick, which does not
# track -- it oscillates or runs away. Worth saying out loud, because a too-small `Ti` looks
# like a tuning problem rather than a broken one.
function _warn_integral_time(Ti, Ts)
    Ti > 5 * Ts || @warn """
        Ti = $Ti is not much larger than Ts = $Ts, so the integrator gain Ts/Ti = \
        $(round(Ts / Ti, digits = 3)) per tick is very aggressive; the velocity loop will \
        not settle. Use Ti of at least a few tens of sample times.""" maxlog=1
    return
end

"""
    generate_friction_controller(; Ts=0.005, log_file=FRICTION_LOG_FILE, overrides...)

Compile the friction experiment to a SynchJulia node: build `FurutaFriction` on a
`PeriodicClock` at sample time `Ts`, with its `DataLogger` writing to `log_file`, and
`stkcompile` it.

Returns what [`compile_program`](@ref) returns: `(; compiled, tuning_defaults, log, Ts, ...)`. The
node argument order is `(tick::Bool, gains::TuningGains, auto::AutoPars)` and the outputs are
`(row, w_ref, shoulder_angle, shoulder_velocity, u_applied)`.

`log_file` goes to the model, not just to the driver: it is the `DataLogger`'s structural
`filename`, which is how [`FrictionController`](@ref) knows what to open. Pass model
parameters as `overrides` using Dyad's `__`-separated paths, e.g. `w_max = 20.0` for the
model's own parameters or `velocity_pi__wp = 1.0` for one of a sub-component's.

Note that `Ni`, the PI loop's anti-windup gain, has a default expression in `Ti` and so is
resolved once here rather than tracking a later runtime change to `Ti` -- retuning `Ti` by a
large factor is worth a recompile.
"""
function generate_friction_controller(; Ts = 0.005, log_file = FRICTION_LOG_FILE,
                                       param_overrides = nothing, overrides...)
    gen = compile_program(FurutaFriction; name = :friction, Ts,
                          tunables = FRICTION_TUNABLES, outputs = _friction_outputs,
                          log = friction_log(log_file), param_overrides, overrides...)
    _warn_integral_time(gen.tuning_defaults[:Ti], Ts)
    return gen
end

"""
    FrictionController(; Ts=0.005, backend=:julia, log_file=FRICTION_LOG_FILE, K=nothing,
                         Ti=nothing, param_overrides=nothing, overrides...)

A ready-to-call runtime wrapper around the generated friction experiment. Compiles the
program, builds a `SynchExecutable` on `backend` (`:julia` or `:c`), and populates the
parameter structs.

Advance one step with `out = controller()`, which reads the encoders, updates the velocity
reference and the PI loop, writes the motor voltage and appends a row to the log, returning
`(; row, w_ref, shoulder, velocity, u)`. Open the device with [`open_hardware!`](@ref) and the
log with [`open_log!`](@ref) first -- or just call [`run_friction_experiment`](@ref), which
does both.

`K` and `Ti` override the velocity-loop gains at runtime; both default to the model's.
"""
function FrictionController(; Ts = 0.005, backend::Symbol = :julia,
                             log_file = FRICTION_LOG_FILE, K = nothing, Ti = nothing,
                             kwargs...)
    gen = generate_friction_controller(; Ts, log_file, kwargs...)
    Ti === nothing || _warn_integral_time(float(Ti), Ts)
    return make_runtime(gen, FRICTION_OUTPUT_NAMES; backend, gains = (; K, Ti))
end

"""
    friction_sweep_duration(; n_levels=6, t_step=2.0) -> Float64

Seconds one full cycle of the `VelocityStaircase` reference takes: `n_levels` speeds in each
direction, `t_step` seconds each. This is the natural run length for the experiment --
shorter truncates the sweep, longer just repeats it.
"""
friction_sweep_duration(; n_levels = 6, t_step = 2.0) = 2 * float(n_levels) * float(t_step)

"""
    run_friction_experiment(ctrl; Tf, arm_deg=0.0, card_options=nothing)

Run `ctrl` against the physical QUBE for `Tf` seconds and return `(; rows, ticks, log_file,
timing)` — [`run_program!`](@ref) with this experiment's default duration.

`card_options` defaults to `nothing`, i.e. `qube_hw.c`'s own deadband compensation, which is
deliberate: the point of the experiment is to measure the friction *the controller faces*, so
it has to run on the same command-to-torque path the controller does. Turning compensation off
here makes `kc` absorb the amplifier deadband, which then describes a plant that no runs use —
see the note in csrc/qube_hw.h for what that costs.
"""
run_friction_experiment(ctrl::ProgramRuntime; Tf = friction_sweep_duration(), arm_deg = 0.0,
                        card_options = nothing) =
    run_program!(ctrl; Tf, arm_deg, card_options)
