# Code generation for the discrete swing-up controller.
#
# `FurutaHardware` is a purely discrete (clocked) system: the `SwingupWithHoming`
# state machine -- algebraic blocks plus discrete-time velocity estimators
# (`DiscreteDerivative` + `ExponentialFilter`) -- wired between the two hardware
# components. SynchToolkit's `stkcompile` compiles it into a standalone
# synchronous node and emits Julia- or C-executable code for it.
#
# The generated node has the runtime signature
#     (shoulder_angle, elbow_angle, u_applied) = step(tick, gains, auto)
# where `tick` is the periodic-clock trigger (`Bool`), `gains` carries the tunable
# LQR gains `L1..L4` and the stabilizer saturation `umax`, and `auto` carries the
# remaining model parameters (resolved to their model values).
#
# The node does its own I/O: `HardwareMeasurement` reads the encoders and
# `HardwareCommand` writes the motor voltage, both by calling into csrc/qube_hw.c
# (see src/hardware_io.jl). So the caller supplies nothing but a clock tick, and
# the three outputs are what the program measured and applied -- everything needed
# for logging. The same is true of the exported C: `run_hardware.c` is a bare
# timing loop with no hardware calls of its own.

using SynchToolkit
using SynchToolkit: ClockedOutput, InputClock, ParametersStruct
import SynchCompiler
import SynchJulia
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using DiscreteComponents: PeriodicClock
using OrderedCollections: OrderedDict
using ControlSystemsMTK: named_ss
using ControlSystemsBase: c2d, ss, lqr
using LinearAlgebra: Diagonal, I, pinv

export generate_swingup_controller, export_swingup_c, SwingupController, design_lqr,
       launch_live_plot, deploy_hardware_harness, run_hardware_harness_remote

"""
    design_lqr(; Ts=0.005, Q1=[1000.0, 10.0, 1.0, 1.0], Q2=100.0) -> L::Vector{Float64}

Design the LQR state-feedback gain `L` for the `LQRstabilizer`. The `FurutaSwingup`
plant is linearized about the upright equilibrium (with the controller loop opened at
the `u_plant`/`shoulder_y`/`elbow_y` analysis points), discretized at sample time `Ts`,
and an LQR problem is solved with state penalty `Q1` and control penalty `Q2`.

`Q1` is the diagonal of the state cost in the order `[shoulder_angle, elbow_angle,
shoulder_velocity, elbow_velocity]`; `Q2` is the scalar control cost. The returned `L`
is the 4-element gain expected by `LQRstabilizer.L`.

The friction terms the controller compensates are deactivated for the linearization; the
first-order term is kept, because the feedforward leaves it alone (it carries the motor's
back-EMF). Pass `friction = true` to design against everything instead.
"""
function design_lqr(; Ts = 0.005, Q1 = [1000.0, 10.0, 1.0, 1.0], Q2 = 100.0,
                     friction::Bool = false)
    @named model = FurutaSwingup()
    ssys = ModelingToolkit.toggle_namespacing(model, false)
    op = Dict(
        ssys.qubependulum.elbow_joint.phi    => pi,
        ssys.qubependulum.shoulder_joint.phi => 0.0,
        ssys.qubependulum.elbow_joint.w      => 0.0,
        ssys.qubependulum.shoulder_joint.w   => 0.0,
        ssys.qubependulum.voltage            => 0.0,
        ssys.elbow_sampler.u    => 0.0,
        ssys.shoulder_sampler.u => 0.0,
    )
    # Zero, in the operating point, exactly the friction terms the controller's feedforward
    # cancels (`SwingupCatch.friction_ff`, built with `kv = 0`) — designing against a
    # disturbance that is already being removed would be designing for the wrong plant.
    #
    # `kv` is deliberately kept: it is friction *and* back-EMF together, the feedforward
    # leaves it alone, so the stabilizer really does face it. `w_tanh` is kept too — it is a
    # divisor. What this removes matters more than it sounds: the smoothed Coulomb term
    # contributes `kc / w_tanh` of damping at zero velocity, several times everything else
    # on the axis, so leaving it in dominates the linearization about the upright.
    friction || for p in (ssys.qubependulum.friction.kc, ssys.qubependulum.friction.k2,
                          ssys.qubependulum.friction.k3)
        op[p] = 0.0
    end
    # Outputs define the order of the `Q1` diagonal below.
    outputs = [
        ssys.qubependulum.shoulder_angle,
        ssys.qubependulum.elbow_angle,
        ssys.qubependulum.shoulder_joint.w,
        ssys.qubependulum.elbow_joint.w,
    ]
    P = named_ss(model, [ssys.u_plant], outputs;
        op,
        loop_openings = [ssys.u_plant, ssys.shoulder_y, ssys.elbow_y],
        warn_empty_op = true,
        additional_passes = [SynchToolkit.compile_lustre],
        MultibodyComponents.linsys...,
    )
    Pd = c2d(ss(P), Ts)
    Q1mat = P.C' * Diagonal(collect(float.(Q1))) * P.C
    Q2mat = float(Q2) * I(1)
    return vec(lqr(Pd, Q1mat, Q2mat) * pinv(P.C))
end

# Resolve the two stabilizer values carried by the runtime-settable `TuningGains` struct,
# returning `(Lsym, umaxsym, Ldef, umaxdef)`. All other parameters end up in the
# autogenerated `AutoPars` struct, whose field defaults SynchToolkit resolves from the
# system itself; only static structs carry no defaults, so these two are resolved here with
# `evaluate_varmap!`, which substitutes the default map into itself to a fixed point
# (values may be expressions of other parameters). Resolve against the full pre-partition
# default map: expression defaults may reference parameters the compiled partition drops
# (SynchToolkit#144).
function _resolve_stabilizer_gains(sys, lqr_stab)
    SymT = ModelingToolkit.SymbolicT
    dv = Dict{SymT, SymT}(ModelingToolkit.default_values(ModelingToolkit.expand_connections(sys)))
    Lsym, usym = ModelingToolkit.unwrap(lqr_stab.L), ModelingToolkit.unwrap(lqr_stab.umax)
    ModelingToolkit.evaluate_varmap!(dv, [Lsym, usym])
    Ldef    = Float64.(vec(collect(Symbolics.value(dv[Lsym]))))
    umaxdef = Float64(Symbolics.value(dv[usym]))
    return Lsym, usym, Ldef, umaxdef
end

"""
    generate_swingup_controller(; Ts=0.005, L=nothing, umax=nothing, overrides...)

Compile the swing-up controller to a SynchJulia node: build `FurutaHardware` -- the
`SwingupWithHoming` state machine wired between `HardwareMeasurement` and
`HardwareCommand` on a `PeriodicClock` at sample time `Ts` -- and `stkcompile` it.

Returns a named tuple `(; topmod, Ldef, umaxdef)` where `topmod` is the evaluated runtime
module (with `topmod.top`, `topmod.TuningGains`, `topmod.AutoPars`, `topmod.executable`)
and `Ldef`/`umaxdef` are the resolved stabilizer gain/saturation values used to populate
the runtime-settable `TuningGains` struct.

The node argument order is `(tick::Bool, gains::TuningGains, auto::AutoPars)` and the
outputs are `(shoulder_angle, elbow_angle, u_applied)`: the node reads the encoders and
writes the motor itself, so the angles are results rather than inputs. The LQR gains `L`
and the stabilizer `umax` are the runtime-settable `TuningGains` fields, the rest are baked
into `AutoPars`.

The controller uses the `SwingupCatch` model's tuned defaults (energy-swingup gain,
arm-centering, LQR gains and saturations), which are set for the QuanserInterface-
matched `QubePendulum` plant. Pass `overrides` using Dyad's `__`-separated paths
(e.g. `control_system__runtime__swingup_catch__energyswingup__umax = 2.5`) to change them;
`L`
(a 4-vector) and `umax` are shorthands for the two stabilizer paths.
"""
function generate_swingup_controller(; Ts = 0.005, L = nothing, umax = nothing, overrides...)
    # The node calls into csrc/qube_hw.c, so the library has to exist before the
    # `:c` backend links it (the Julia backend only needs it at call time).
    ensure_qube_hw()
    # `L`/`umax` are the two knobs worth a dedicated argument; route them through the same
    # `__`-path override mechanism as everything else so the model needs no wrapping.
    stab = :control_system__runtime__swingup_catch__lqrstabilizer__
    kw = Dict{Symbol, Any}(overrides)
    L    === nothing || (kw[Symbol(stab, :L)]    = collect(float.(L)))
    umax === nothing || (kw[Symbol(stab, :umax)] = float(umax))
    sys = FurutaHardware(; name = :controller, Ts, kw...)
    # `sys` is the root, so its own name is not part of the flattened symbol names.
    # Reach for symbols through the un-namespaced view so they match what
    # `default_values` and `stkcompile` see (same reason as in `design_lqr`).
    nsys = ModelingToolkit.toggle_namespacing(sys, false)
    lqr_stab = nsys.control_system.runtime.swingup_catch.lqrstabilizer

    Lsym, usym, Ldef, umaxdef = _resolve_stabilizer_gains(sys, lqr_stab)

    gain_syms = OrderedDict{Any, Symbol}(Lsym => :L, usym => :umax)
    inputs = [
        InputClock(ModelingToolkit.Clock(Ts)),
        ParametersStruct(; arg_name = :gains, struct_name = :TuningGains,
                           parameters = gain_syms, generated = false),
        ParametersStruct(; arg_name = :auto, struct_name = :AutoPars),
    ]
    # Output order defines the fields of what the controller returns. Neither `name` nor
    # `clock` may be passed here: `name` desynchronises the declared and assigned Lustre
    # names, and `clock` hits a missing branch in SynchToolkit's `build_output`.
    outputs = [
        ClockedOutput(nsys.measurement.shoulder_angle),
        ClockedOutput(nsys.measurement.elbow_angle),
        ClockedOutput(nsys.command.u_applied),
    ]
    @info "Running stkcompile"
    topmod = SynchToolkit.stkcompile(sys; inputs, outputs)
    return (; topmod, Ldef, umaxdef)
end

"""
    SwingupController(; Ts=0.005, backend=:julia, L=nothing, umax=nothing, overrides...)

A ready-to-call runtime wrapper around the generated swing-up controller. Compiles
the controller, builds a `SynchExecutable` on `backend` (`:julia` or `:c`), and
populates the parameter structs.

Advance one control step with `out = controller()`, which reads the encoders, runs the
state machine, writes the motor voltage and returns `(; shoulder, elbow, u)` -- what it
measured and what it applied. Point it at a device with [`open_hardware!`](@ref) (real
QUBE) or [`bind_hardware!`](@ref) (a simulator) first.

`L` overrides the LQR feedback gains `[L1, L2, L3, L4]` and `umax` the stabilizer
saturation; both default to the model's tuned values.
"""
struct SwingupController{E, G, A}
    exe::E
    gains::G
    auto::A
end

function SwingupController(; Ts = 0.005, backend::Symbol = :julia,
                            L = nothing, umax = nothing, kwargs...)
    gen = generate_swingup_controller(; Ts, kwargs...)
    return _make_runtime(gen; backend, L, umax)
end

# Consume the compiled runtime module: construct the `TuningGains`/`AutoPars` parameter
# objects (applying the LQR-gain / saturation overrides `L`/`umax`; `nothing` keeps the
# model default), optionally build a `SynchExecutable` on `backend`, and optionally export
# the C sources into `export_dir`. These objects are what the runtime hands to the node's
# `step` (as opaque pointers in the C backend); their in-memory field bytes are exactly
# what the exported C reads at its baked-in `fieldoffset`s.
#
# `stkcompile` evaluates the runtime module in a newer world than this frame, so per
# SynchToolkit's consumption contract the boundary is crossed exactly once, here: the
# single `@invokelatest` around `consume` raises the world for all module member reads and
# constructor calls, and everything returned (values, types, the executable) is world-safe
# to use from any frame afterwards.
function _instantiate(gen; L = nothing, umax = nothing,
                      backend::Union{Nothing, Symbol} = nothing, export_dir = nothing)
    Lv = collect(float.(something(L, gen.Ldef)))
    umaxv = float(something(umax, gen.umaxdef))
    function consume(m)
        gains = m.TuningGains(; L = Lv, umax = umaxv)
        # pass the static struct: AutoPars defaults may be expressions of its fields
        auto = m.AutoPars(gains)
        exe = backend === nothing ? nothing : m.executable(backend)
        export_dir === nothing || SynchCompiler.export_c(export_dir, m.top)
        return (; gains, auto, exe, SG = m.TuningGains, AP = m.AutoPars)
    end
    return Base.@invokelatest consume(gen.topmod)
end

# Build a runtime from an already-compiled controller (avoids recompiling).
function _make_runtime(gen; backend::Symbol = :julia, L = nothing, umax = nothing)
    r = _instantiate(gen; L, umax, backend)
    return SwingupController(r.exe, r.gains, r.auto)
end

"""
    (c::SwingupController)(; tick=true) -> (; shoulder, elbow, u)

Advance the controller one step: it reads the encoders, runs the state machine and
writes the motor voltage, then reports what it measured and applied. Angles are in
radians, with the pendulum-upright reference at π (matching `QuanserInterface`).

With `tick=false` the clock does not fire, so no hardware is touched and the
returned values are meaningless (`nothing`).
"""
# A held executable keeps stepping the code it was built with (SynchJulia ≥ 0.4), so
# `step!`/`reset!` are world-safe from any frame — no `invokelatest` in the hot path.
function (c::SwingupController)(; tick::Bool = true)
    out = SynchJulia.step!(c.exe, tick, c.gains, c.auto)
    v = values(out)
    return (; shoulder = v[1], elbow = v[2], u = v[3])
end

# Resets the node's state machine (back to homing) and the I/O call counters; the device
# itself stays open.
function SynchToolkit.reset!(c::SwingupController)
    SynchToolkit.reset!(c.exe)
    reset_hardware_counters!()
    return c
end

"""
    export_swingup_c(dir; Ts=0.005, L=nothing, umax=nothing, Tf=10.0, arm_deg=0.0, overrides...)

Generate the swing-up controller and export standalone C sources into `dir`: the
SynchToolkit node (`top.c`, `top.h`, `top.pc`, `synchjulia.h`), the hardware I/O the node
calls into (`qube_hw.c`, `qube_hw.h`, copied from `csrc/`), and a runnable control loop
(`run_hardware.c`, `Makefile`) — see [`emit_hardware_harness`](@ref). `L`/`umax` override
the LQR gains and stabilizer saturation; `Tf` is the run duration and `arm_deg` the arm's
physical angle at start-up (see [`open_hardware!`](@ref)), both baked into the harness.

The exported `top.c` contains `extern double qube_hw_measure(double arg1);` and friends
plus named call sites, so the node performs its own encoder reads and motor writes and the
harness is only a timing loop. That is the same code path the Julia backend takes — one
controller definition, one hardware implementation, three targets.

Returns `(; dir, topmod, mangled, gains, auto)` where `mangled` is the base symbol name
for the emitted `<mangled>_step` / `<mangled>_reset` functions and `gains`/`auto` are the
parameter objects embedded into the harness.
"""
function export_swingup_c(dir; Ts = 0.005, L = nothing, umax = nothing, Tf = 10.0,
                          arm_deg = 0.0, card_options = nothing, kwargs...)
    gen = generate_swingup_controller(; Ts, L, umax, kwargs...)
    r = _instantiate(gen; L, umax, export_dir = dir)
    mangled = SynchCompiler.mangle("top", Bool, r.SG, r.AP)
    for f in ("qube_hw.c", "qube_hw.h")
        cp(joinpath(dirname(QUBE_HW_SRC), f), joinpath(dir, f); force = true)
    end
    emit_hardware_harness(dir; Ts, Tf, arm_deg, card_options, mangled, r.gains, r.auto,
                          outfields = _out_field_names(joinpath(dir, "top.h"), mangled))
    # `export_c` copies `synchjulia.h` out of the package depot, which is read-only, and
    # preserves its mode. Everything here is a build output, so leave nothing unwritable:
    # otherwise a later export or `scp` into the same directory cannot replace it. Only the
    # user-write bit is added, so an executable stays executable.
    for f in readdir(dir; join = true)
        isfile(f) || continue
        m = filemode(f)
        (m & 0o200) == 0 && chmod(f, m | 0o200)
    end
    return (; dir, gen.topmod, mangled, r.gains, r.auto)
end

# Field names of the node's output struct, in output order, read back out of the emitted
# header rather than reconstructed. SynchCompiler derives them from the fully namespaced
# symbol names and sanitizes them for C, so they follow the model's component hierarchy —
# reading them here keeps the harness correct if that hierarchy changes.
function _out_field_names(header, mangled)
    src = read(header, String)
    m = match(Regex("typedef struct \\{([^}]*)\\}\\s*$(mangled)_out;"), src)
    m === nothing && error("could not find the $(mangled)_out struct in $header")
    # The struct also carries a `has_<name>` bool per clocked output (set when that
    # output's clock fired); the values are the `double` fields.
    names = [String(f[1]) for f in eachmatch(r"^\s*double\s+(\w+);"m, m[1])]
    length(names) == 3 || error("expected 3 double outputs in $(mangled)_out, found $names")
    return names
end

# Raw little-endian bytes of a runtime parameter object (`TuningGains`/`AutoPars`), packed
# into 64-bit words. Both are mutable structs whose fields are stored inline, so
# `pointer_from_objref` points at the flat field-data region the exported C reads via its
# baked-in `fieldoffset`s. Packing to `uint64_t` guarantees the 8-byte alignment the C
# pointer casts require (the data region is a whole number of 8-byte fields; pad defensively).
function _param_words(obj)
    n = sizeof(typeof(obj))
    npad = cld(n, 8) * 8
    bytes = zeros(UInt8, npad)
    GC.@preserve obj bytes begin
        unsafe_copyto!(pointer(bytes), Ptr{UInt8}(pointer_from_objref(obj)), n)
    end
    return reinterpret(UInt64, bytes)
end

"""
    emit_hardware_harness(dir; Ts, Tf, arm_deg, card_options, mangled, gains, auto, outfields)

Write `run_hardware_config.h` into `dir` and copy `csrc/run_hardware.c` and `csrc/Makefile`
alongside it, next to the exported `top.c`/`top.h` and `qube_hw.c`/`qube_hw.h`.

The control loop itself is a checked-in C file, not a string built here: only the values
that depend on the compiled controller are generated, as a small header the loop includes.
Those are the mangled node symbols (`_step`/`_reset`/`_mem`/`_out`), the names of the
fields in its output struct, the sample time, the run duration, the arm's start-up angle,
the card options, and the parameter blocks. Keeping the C in a real file means it gets
editor tooling and cannot acquire escaping bugs from being embedded in Julia.

The loop is only timing and logging: `qube_hw_open(QUBE_HW_MODE_HIL, ...)` (home the arm
and let the pendulum hang before starting), then every `Ts` seconds a single `_step` call,
which reads the encoders, runs the state machine and writes the motor itself through
`qube_hw.c`. The `gains`/`auto` parameter objects are serialized to raw bytes and embedded
so `_step` sees the exact byte layout it was compiled against.

It writes a tab-separated `run_hardware.csv` matching `test/hardware_swingup.jl`'s
`swingup.csv` layout — columns `time, shoulder_angle, elbow_angle, control_input` (load with
`D = readdlm("run_hardware.csv", skipstart=1)'` and pass to `plotD`) plus two timing
diagnostics `dt` (achieved period) and `exec` (loop-body duration). Timing mirrors the Julia
`@periodically` loop (run body, sleep the remainder of `Ts`); a one-line timing summary is
printed to stderr on exit.
"""
function emit_hardware_harness(dir; Ts, Tf, mangled, gains, auto, arm_deg = 0.0,
                               card_options = nothing,
                               outfields = ["measurement_shoulder_angle_t_",
                                            "measurement_elbow_angle_t_",
                                            "command_u_applied_t_"])
    words(obj) = join(("0x" * string(w; base = 16, pad = 16) * "ULL" for w in _param_words(obj)), ", ")
    config = """
    /* Auto-generated by QuanserComponents.emit_hardware_harness — do not edit by hand.
     * Everything here depends on the compiled controller; run_hardware.c is a normal
     * checked-in C file that includes this. */
    #ifndef RUN_HARDWARE_CONFIG_H
    #define RUN_HARDWARE_CONFIG_H

    #include <stdint.h>

    /* Mangled entry points of the generated node. */
    #define QUBE_STEP  $(mangled)_step
    #define QUBE_RESET $(mangled)_reset
    #define QUBE_MEM   $(mangled)_mem
    #define QUBE_OUT   $(mangled)_out

    /* Fields of the node's output struct, in the order the outputs were declared. */
    #define QUBE_OUT_SHOULDER $(outfields[1])
    #define QUBE_OUT_ELBOW    $(outfields[2])
    #define QUBE_OUT_U        $(outfields[3])

    #define QUBE_TS $(Float64(Ts))            /* sample time [s]  */
    #define QUBE_TF $(Float64(Tf))            /* run duration [s] */
    /* Where the arm physically sits at start-up; added to every shoulder reading so the
     * arm need not be moved to its home position first. See qube_hw.h. */
    #define QUBE_ARM0 $(deg2rad(Float64(arm_deg)))  /* [rad] */
    $(card_options === nothing ? "/* card options left at the qube_hw.c default */" :
      "#define QUBE_CARD_OPTIONS \"$(card_options)\"")

    /* Parameter blocks that the generated `_step` reads as opaque pointers: the raw bytes
     * of the Julia TuningGains (LQR gains L[4] + stabilizer saturation umax) and AutoPars
     * (all other model constants), serialized so the byte offsets match top.c exactly. */
    static const uint64_t gains_words[] = { $(words(gains)) };
    static const uint64_t auto_words[]  = { $(words(auto)) };
    #define GAINS_PTR ((int64_t)(intptr_t)gains_words)
    #define AUTO_PTR  ((int64_t)(intptr_t)auto_words)

    #endif /* RUN_HARDWARE_CONFIG_H */
    """
    write(joinpath(dir, "run_hardware_config.h"), config)
    for f in ("run_hardware.c", "Makefile")
        cp(joinpath(dirname(QUBE_HW_SRC), f), joinpath(dir, f); force = true)
    end
    return (; c = "run_hardware.c", config = "run_hardware_config.h", makefile = "Makefile")
end

"""
    compile_hardware_harness(dir; quanser_dir="/opt/quanser/hil_sdk") -> exe_path

Compile the emitted `run_hardware.c` + `top.c` + `qube_hw.c` in `dir` into a
`run_hardware` executable, linking the Quanser HIL SDK. Uses the system C compiler (`cc`,
falling back to `gcc`). Linking uses the static SDK libraries, so this does not require the
hardware to be connected — it doubles as a build check for the generated sources, including
that the node's `extern qube_hw_*` declarations resolve against `qube_hw.c`.
"""
function compile_hardware_harness(dir; quanser_dir = QUANSER_HIL_DIR)
    @info "Compiling hardware harness"
    cc = Sys.which("cc")
    cc === nothing && (cc = Sys.which("gcc"))
    cc === nothing && error("compile_hardware_harness: no C compiler (cc/gcc) found on PATH")
    sdk = quanser_sdk_flags(; quanser_dir)
    sdk.found || error("""
        compile_hardware_harness: no Quanser HIL SDK found (looked for
        $(joinpath(quanser_dir, "include", "hil.h")) and /usr/include/quanser/hil.h).""")
    exe = abspath(joinpath(dir, "run_hardware"))
    args = [cc, "-I$dir", sdk.cflags..., "-O2", "-Wall", "-DQUBE_HW_HAVE_HIL",
            joinpath(dir, "run_hardware.c"), joinpath(dir, "top.c"),
            joinpath(dir, "qube_hw.c"), "-o", exe,
            sdk.ldflags..., QUANSER_LIBS...]
    run(Cmd(args))
    return exe
end

"""
    launch_live_plot(dir; cmd="kst2", config="kst2config.kst", wait_for_log=5.0) -> Process | Nothing

Start a live plotter on the log the hardware harness writes, for watching the run as it
happens. Returns the running process, or `nothing` if it could not be started.

`cmd` is the viewer executable and `config` its session file (a relative path resolves
against `dir`); the default pair is [kst2](https://kst-plot.kde.org/) reading the
`run_hardware.csv` that `run_hardware` appends to. The viewer is launched with its working
directory set to `dir`, so a session file referring to the log by relative path works
wherever `dir` is.

Call this *after* starting the harness but *before* waiting on it — the plotter has to come
up alongside the run, not after it. It waits up to `wait_for_log` seconds for the log file
to appear first, since a viewer pointed at a missing file typically gives up rather than
retrying.

This never throws: a missing viewer, a missing session file or a failed launch is reported
as a warning and returns `nothing`, because losing the plot is not a reason to abandon a
hardware run. The process outlives this call and is not reaped — `kill` it when done.
"""
function launch_live_plot(dir; cmd = "kst2", config = "kst2config.kst",
                          wait_for_log = 5.0)
    dir = abspath(dir)
    cfg = isabspath(config) ? config : joinpath(dir, config)
    if Sys.which(cmd) === nothing
        @warn "live plot: `$cmd` not found on PATH, skipping"
        return nothing
    end
    if !isfile(cfg)
        @warn "live plot: session file not found, skipping" config = cfg
        return nothing
    end
    # A viewer left over from an earlier run is the most confusing failure here: it holds
    # the previous log open, never updates again, and is indistinguishable from the new
    # window. Retire it rather than adding to the pile.
    _close_stale_plotters(cmd, cfg)
    csv = joinpath(dir, "run_hardware.csv")
    t0 = time()
    while !isfile(csv) && time() - t0 < wait_for_log
        sleep(0.05)
    end
    isfile(csv) ||
        @warn "live plot: $(basename(csv)) has not appeared, starting $cmd anyway" wait_for_log
    _check_plot_fields(cfg, csv)
    try
        return run(Cmd(`$cmd $cfg`; dir); wait = false)
    catch e
        @warn "live plot: could not start `$cmd`" exception = e
        return nothing
    end
end

# A kst session names the columns it plots, and names them *as the data source reports
# them* — for an ASCII source with a header, that is the header text. Get one wrong and kst
# draws an empty plot in silence: the vector simply resolves to nothing, and because the
# x vector is shared by every curve, one stale name empties the whole window. That is a
# genuinely hard failure to read at the GUI, so say it here instead.
#
# Session files accumulate vectors as they are edited, and stale ones referring to columns
# that no longer exist are normal and harmless, so a missing field is only worth reporting
# when *nothing* matches — that is the case that means a blank window.
# Terminate viewers already running on this session file. Matches on the config path too,
# so unrelated instances of the same program are left alone.
function _close_stale_plotters(cmd, cfg)
    try
        for line in eachsplit(read(`pgrep -af $(basename(cmd))`, String), '\n')
            occursin(cfg, line) || continue
            pid = tryparse(Int, first(split(line)))
            (pid === nothing || pid == getpid()) && continue
            @info "live plot: retiring a viewer left over from an earlier run" pid
            run(`kill $pid`; wait = false)
        end
    catch
        # pgrep missing, or nothing matched: nothing to clean up.
    end
    return
end

function _check_plot_fields(config, csv)
    (isfile(config) && isfile(csv)) || return
    header = split(strip(first(eachline(csv))), '\t')
    fields = Set(m.captures[1] for m in eachmatch(r"field=\"([^\"]*)\"", read(config, String)))
    delete!(fields, "INDEX")                    # kst's built-in sample index
    isempty(fields) && return
    matched = intersect(fields, Set(header))
    isempty(matched) && @warn """
        live plot: none of the columns $(basename(config)) plots exist in $(basename(csv)),
        so the plot will come up empty. kst names ASCII fields from the header line, so the
        session must use those names.""" session_fields = sort(collect(fields)) log_header = header
    return
end

# Files `export_swingup_c` writes that the target needs in order to build and run.
const HARNESS_FILES = ("run_hardware.c", "run_hardware_config.h", "Makefile",
                       "top.c", "top.h", "synchjulia.h", "qube_hw.c", "qube_hw.h")

"""
    deploy_hardware_harness(dir; host, remote_dir="furuta_c", ssh=`ssh`, scp=`scp`) -> remote_dir

Copy the exported sources to `host` and build them there. Returns `remote_dir`.

Building on the target rather than cross-compiling is deliberate: the Quanser SDK is
installed on the target (headers, libraries and the board driver plugin for its
architecture), so a cross-toolchain here would have to reproduce that sysroot to gain
nothing. The emitted `Makefile` detects the SDK layout, so the same sources build on an
x86 host with the older HIL SDK and on a Raspberry Pi with the `quanser-sdk` packages.

`host` is anything ssh accepts, e.g. `"fredrikb@192.168.1.49"`. Requires
non-interactive ssh (key-based auth) since nothing here can answer a prompt.
"""
function deploy_hardware_harness(dir; host, remote_dir = "furuta_c",
                                 ssh = `ssh`, scp = `scp`)
    @info "Deploying hardware harness"
    missing_files = [f for f in HARNESS_FILES if !isfile(joinpath(dir, f))]
    isempty(missing_files) ||
        error("deploy_hardware_harness: $dir is missing $(join(missing_files, ", ")) — \
               run `export_swingup_c` first")
    @info "Creating remote dir"
    # Clear the destinations too. An earlier deploy may have left `synchjulia.h` mode 444
    # (it originates in the read-only package depot), and `scp` cannot reopen such a file
    # for writing — it fails mid-transfer with "Permission denied".
    remote_files = join(("$remote_dir/$f" for f in HARNESS_FILES), " ")
    run(`$ssh $host "mkdir -p $remote_dir && rm -f $remote_files"`)
    @info "Copying files (scp)"
    run(`$scp $([joinpath(dir, f) for f in HARNESS_FILES]) $host:$remote_dir/`)
    @info "make on remote host"
    run(`$ssh $host "cd $remote_dir && make"`)
    return remote_dir
end

"""
    run_hardware_harness_remote(host, remote_dir; local_dir, stream_log=false, ssh=`ssh`, scp=`scp`) -> csv_path

Run the harness on `host`, blocking for its baked-in duration, then copy
`run_hardware.csv` back into `local_dir` and return the local path.

With `stream_log`, the remote log is followed over ssh into `local_dir` *while the run is
in progress*, so a live plotter watching the local file sees the run as it happens rather
than only the copy made afterwards.
"""
function run_hardware_harness_remote(host, remote_dir; local_dir, stream_log = false,
                                     ssh = `ssh`, scp = `scp`)
    @info "Running hardware harness on remote"
    csv = joinpath(local_dir, "run_hardware.csv")
    remote_csv = "$remote_dir/run_hardware.csv"
    # Start from a clean slate so a stale log cannot be mistaken for this run's.
    run(`$ssh $host rm -f $remote_csv`)
    tail = nothing
    if stream_log
        # Truncate in place rather than `rm`. Unlinking would give the new run a fresh
        # inode, and any viewer already watching the old one keeps reading the unlinked
        # file forever -- a live plot that is frozen and looks exactly like a working one.
        # `open(csv, "w")` below truncates, which is all that is wanted.
        # -F retries until the harness creates the file, so this can start first.
        tail = try
            open(csv, "w") do io
                # `stdbuf -oL`: `tail` block-buffers into a pipe by default, which would
                # re-introduce the ~4 KiB granularity the harness avoids by line-buffering
                # its own log. Falls back to plain `tail` where stdbuf is absent.
                run(pipeline(`$ssh $host "command -v stdbuf >/dev/null && exec stdbuf -oL tail -F -n +1 $remote_csv || exec tail -F -n +1 $remote_csv"`;
                             stdout = io, stderr = devnull); wait = false)
            end
        catch e
            @warn "could not start log streaming; the log will arrive after the run" exception = e
            nothing
        end
    end
    try
        run(`$ssh $host "cd $remote_dir && ./run_hardware"`)
    finally
        tail === nothing || kill(tail)
    end
    # Copy the finished log back *through* the existing file rather than over it: `scp`
    # would create a new inode and freeze any viewer watching this path, right at the
    # moment the interesting part of the run finished. When the log was streamed this is
    # only a consistency check anyway -- the local copy is already complete.
    tmp = csv * ".fetch"
    run(`$scp $host:$remote_csv $tmp`)
    open(csv, "w") do io
        write(io, read(tmp))
    end
    rm(tmp; force = true)
    return csv
end

"""
    run_hardware_harness(exe) -> csv_path

Execute the compiled `run_hardware` binary, blocking for its baked-in run duration `Tf`.
The binary controls the physical pendulum and writes `run_hardware.csv` (columns `time,
shoulder_angle, elbow_angle, control_input`) in its working directory; that path is returned.
"""
function run_hardware_harness(exe)
    @info "Running hardware harness"
    exe = abspath(exe)
    dir = dirname(exe)
    # Run with the child's working directory set to `dir` (not the current process's) so
    # the binary — addressed by absolute path — is found and `run_hardware.csv` lands in `dir`.
    run(Cmd(`$exe`; dir))
    return joinpath(dir, "run_hardware.csv")
end
