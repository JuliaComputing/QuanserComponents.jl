# The swing-up controller as a synchronous program: what is specific to it.
#
# `FurutaHardware` is a purely discrete (clocked) system: the `SwingupWithHoming` state
# machine -- algebraic blocks plus discrete-time velocity estimators (`DiscreteDerivative` +
# `ExponentialFilter`) -- wired between the two hardware components, with a `DataLogger`
# writing a row per tick. SynchToolkit's `stkcompile` compiles it into a standalone
# synchronous node and emits Julia- or C-executable code for it.
#
# The generated node has the runtime signature
#     (row, shoulder_angle, elbow_angle, u_applied) = step(tick, gains, auto)
# where `tick` is the periodic-clock trigger (`Bool`), `gains` carries the tunable LQR gains
# `L1..L4` and the motor saturation `umax`, and `auto` carries the remaining model parameters
# (resolved to their model values).
#
# The node does its own I/O and its own logging: `HardwareMeasurement` reads the encoders,
# `HardwareCommand` writes the motor voltage and `DataLogger` appends the row, all by calling
# into csrc/qube_hw.c and csrc/qube_log.c (see src/hardware_io.jl and src/data_log.jl). So
# the caller supplies nothing but a clock tick, and the outputs are for inspection rather
# than for logging. The same is true of the exported C: `run_hardware.c` is a bare timing
# loop with no hardware calls and no logging of its own.
#
# Everything here that is not about *this* program lives in program.jl (compiling, the
# runtime, the in-process loop) and harness.jl (C export, deployment, live plotting).

using ControlSystemsMTK: named_ss
using ControlSystemsBase: c2d, ss, lqr
using LinearAlgebra: Diagonal, I, pinv

export generate_swingup_controller, export_swingup_c, SwingupController, design_lqr,
       swingup_log

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

# ---------------------------------------------------------------------------
## The program
# ---------------------------------------------------------------------------
"The log `FurutaHardware` writes, in `SWINGUP_LOG_COLUMNS` order. `file` defaults to `SWINGUP_LOG_FILE`."
swingup_log(file = SWINGUP_LOG_FILE) = ProgramLog(file, SWINGUP_LOG_COLUMNS)

# The two parameters that stay settable at runtime, and the fields they take in
# `TuningGains`. Both are root parameters of `FurutaHardware`: `umax` is bound down into the
# stabilizer and the command clamp with `final`, `L` is the stabilizer's own (nothing binds
# it). A `ParametersStruct` field has to be unbound, which is what makes the root the right
# place for `umax` — see `resolve_tunables`.
const SWINGUP_TUNABLES = OrderedDict{Any, Symbol}(
    (nsys -> nsys.control_system.runtime.swingup_catch.lqrstabilizer.L) => :L,
    (nsys -> nsys.umax) => :umax,
)

# Node outputs, in the order the runtime reports them. `row` first so the cheapest check --
# did the program write one row per tick -- is the first thing available.
_swingup_outputs(nsys) = [nsys.logger.row, nsys.measurement.shoulder_angle,
                          nsys.measurement.elbow_angle, nsys.command.u_applied]
const SWINGUP_OUTPUT_NAMES = (:row, :shoulder, :elbow, :u)

"""
    generate_swingup_controller(; Ts=0.005, log_file=SWINGUP_LOG_FILE, L=nothing, umax=nothing, overrides...)

Compile the swing-up controller to a SynchJulia node: build `FurutaHardware` -- the
`SwingupWithHoming` state machine wired between `HardwareMeasurement` and `HardwareCommand`,
with a `DataLogger`, on a `PeriodicClock` at sample time `Ts` -- and `stkcompile` it.

Returns what [`compile_program`](@ref) returns: `(; topmod, tuning_defaults, log, Ts)`. The
node argument order is `(tick::Bool, gains::TuningGains, auto::AutoPars)` and the outputs are
`(row, shoulder_angle, elbow_angle, u_applied)`: the node reads the encoders and writes the
motor itself, so the angles are results rather than inputs. The LQR gains `L` and the motor
saturation `umax` are the runtime-settable `TuningGains` fields, the rest are baked into
`AutoPars`.

`log_file` goes to the model, not just to the driver: it is the `DataLogger`'s structural
`filename`, so the component that writes the file and the call that opens it cannot disagree
about which file that is.

The controller uses the `SwingupCatch` model's tuned defaults (energy-swingup gain,
arm-centering, LQR gains and saturations), which are set for the `QubePendulum` plant with
the identified parameters. Pass `overrides` using Dyad's `__`-separated paths (e.g.
`control_system__runtime__swingup_catch__energyswingup__umax = 2.5`) to change them; `L` (a
4-vector) and `umax` are the two with a dedicated argument, since they are the tunable ones.
"""
function generate_swingup_controller(; Ts = 0.005, log_file = SWINGUP_LOG_FILE,
                                      L = nothing, umax = nothing, param_overrides = nothing,
                                      overrides...)
    gen = compile_program(FurutaHardware; name = :controller, Ts,
                          tunables = SWINGUP_TUNABLES, outputs = _swingup_outputs,
                          log = swingup_log(log_file), param_overrides, overrides...)
    # `L`/`umax` given here override the model's resolved values at instantiation, not at
    # compile time, so a retune needs no recompile.
    return (; gen..., gains = (; L, umax))
end

"""
    SwingupController(; Ts=0.005, backend=:julia, log_file=SWINGUP_LOG_FILE, L=nothing, umax=nothing, overrides...)

A ready-to-call runtime wrapper around the generated swing-up controller. Compiles the
controller, builds a `SynchExecutable` on `backend` (`:julia` or `:c`), and populates the
parameter structs.

Advance one control step with `out = controller()`, which reads the encoders, runs the state
machine, writes the motor voltage, logs a row and returns `(; row, shoulder, elbow, u)` --
what it measured and what it applied. Point it at a device with [`open_hardware!`](@ref)
(real QUBE) or [`bind_hardware!`](@ref) (a simulator) first, and open the log with
[`open_log!`](@ref) if the rows are wanted; with no log open the logging call is inert.

`L` overrides the LQR feedback gains `[L1, L2, L3, L4]` and `umax` the motor saturation; both
default to the model's tuned values. [`run_program!`](@ref) does the opening, the timing and
the closing for a real run.
"""
function SwingupController(; Ts = 0.005, backend::Symbol = :julia,
                            log_file = SWINGUP_LOG_FILE, L = nothing, umax = nothing,
                            kwargs...)
    gen = generate_swingup_controller(; Ts, log_file, kwargs...)
    return make_runtime(gen, SWINGUP_OUTPUT_NAMES; backend, gains = (; L, umax))
end

"""
    export_swingup_c(dir; Ts=0.005, log_file=SWINGUP_LOG_FILE, L=nothing, umax=nothing, Tf=10.0, arm_deg=0.0, card_options=nothing, overrides...)

Compile the swing-up controller and export it as standalone C into `dir` — see
[`export_program_c`](@ref), which does the work and documents what lands there.

Returns `(; dir, topmod, mangled, files, gains, auto)`.
"""
function export_swingup_c(dir; Ts = 0.005, log_file = SWINGUP_LOG_FILE, L = nothing,
                          umax = nothing, Tf = 10.0, arm_deg = 0.0, card_options = nothing,
                          kwargs...)
    gen = generate_swingup_controller(; Ts, log_file, L, umax, kwargs...)
    r = export_program_c(gen, dir; Tf, arm_deg, card_options, gains = gen.gains)
    return (; r..., gen.topmod)
end
