# The MPC controller as a synchronous program: what is specific to it.
#
# `FurutaMPCHardware` (dyad/furuta_mpc.dyad) is `FurutaHardware` with the swing-up state
# machine replaced by `FurutaMPC`: an `MPCComponents.ACADOSMPC` that swings the pendulum up and
# balances it, solving a constrained nonlinear optimal control problem over the multibody
# `QubePendulum` model every tick. Everything about compiling it, instantiating it and ticking
# it against the rig is shared with the other programs and lives in program.jl; this file
# supplies the prediction model, the log layout, the tunable and the outputs.
#
# The prediction model is the point of the exercise. `ACADOSMPC` normally rebuilds the
# model's Jacobian symbolically, which a `multibody`-compiled model defeats: its right-hand
# side references cached linear solves (MTK `__diffcacheₘₜₖ` parameters) that
# `Symbolics.build_function` cannot reconstruct. The `ForwardDiff` Jacobian backend instead
# evaluates the model numerically through MTK's `generate_control_function` -- whose caches
# are `PreallocationTools.DiffCache`s, hence dual-number safe -- and differentiates that. So
# the very plant model serves as the prediction model, with no hand-written equations and no
# DAE reformulation. The price is that there is no symbolic form to render to C: this
# controller runs on SynchJulia's Julia backend only.

using MPCComponents
using MPCComponents: continuous_dynamics, ContinuousDynamics

export furuta_mpc_dynamics, generate_mpc_controller, MPCController, mpc_log

# ---------------------------------------------------------------------------
## The prediction model
# ---------------------------------------------------------------------------
"""
    furuta_mpc_dynamics(; idparams = identified, jacobian_backend = :forwarddiff) -> ContinuousDynamics

The prediction model of `FurutaMPC`: the `QubePendulum` with parameter set `idparams`,
compiled with `MultibodyComponents.multibody` (input: the motor voltage) and wrapped by
`MPCComponents.continuous_dynamics` with an AD Jacobian backend.

The result is a pure ODE with the four states `FURUTA_MPC_STATES` -- the joint angles
`shoulder_joint.phi`, `elbow_joint.phi` and their derivatives -- and the motor voltage as
its one input, in `qube₊`-prefixed signal names. The plant's `shoulder_angle`/`elbow_angle`
outputs are those joint angles exactly, so the hardware measurements map onto the model
states one to one.

`jacobian_backend` must be an AD backend (`:forwarddiff` or `:finitediff`): the compiled
multibody model references cached linear solves that the `:symbolic` backend cannot
reconstruct (it throws). The model is built once per argument combination and cached, so
constructing several controllers -- or the simulation model next to the hardware program --
does not recompile it.
"""
function furuta_mpc_dynamics(; idparams = identified, jacobian_backend::Symbol = :forwarddiff)
    key = (idparams, jacobian_backend)
    return get!(_MPC_DYNAMICS_CACHE, key) do
        @info "Compiling the Furuta prediction model for the MPC" jacobian_backend
        @named world = MultibodyComponents.World(render = false)
        @named qube = QubePendulum(; idparams)
        model = System(Equation[], t; systems = [world, qube], name = :furuta)
        ssys = MultibodyComponents.multibody(model; inputs = [qube.voltage])
        continuous_dynamics(ssys; inputs = [qube.voltage], jacobian_backend)
    end
end
const _MPC_DYNAMICS_CACHE = Dict{Any, ContinuousDynamics}()

# ---------------------------------------------------------------------------
## The program
# ---------------------------------------------------------------------------
"The log `FurutaMPCHardware` writes, in `MPC_LOG_COLUMNS` order. `file` defaults to `MPC_LOG_FILE`."
mpc_log(file = MPC_LOG_FILE) = ProgramLog(file, MPC_LOG_COLUMNS)

# The one runtime-settable parameter: the command clamp before the amplifier, a root parameter
# of `FurutaMPCHardware` bound down into `HardwareCommand` with `final` (see `resolve_tunables`
# for why the root is the right place). The MPC's own weights are tunable parameters of
# `ACADOSMPC` too, but arrays of a shape `ParametersStruct` has no field type for; they are set
# at construction with `overrides` instead (`control_system__Q1 = ...`).
const MPC_TUNABLES = OrderedDict{Any, Symbol}(
    (nsys -> nsys.command_umax) => :command_umax,
)

# Node outputs, in the order the runtime reports them: the swing-up program's four, then
# acados' status.
_mpc_outputs(nsys) = [nsys.logger.row, nsys.measurement.shoulder_angle,
                      nsys.measurement.elbow_angle, nsys.command.u_applied,
                      nsys.control_system.exitflag]
const MPC_OUTPUT_NAMES = (:row, :shoulder, :elbow, :u, :exitflag)

"""
    generate_mpc_controller(; Ts=0.01, Np=60, log_file=MPC_LOG_FILE, dynamics=furuta_mpc_dynamics(), overrides...)

Compile the MPC controller to a SynchJulia node: build `FurutaMPCHardware` -- the `ACADOSMPC`
swing-up and balancing controller wired between `HardwareMeasurement` and `HardwareCommand`,
with a `DataLogger`, on a `PeriodicClock` at sample time `Ts` -- and `stkcompile` it.

Returns what [`compile_program`](@ref) returns. The node's argument order is
`(tick::Bool, gains::TuningGains, auto::AutoPars)` and the outputs are `(row, shoulder_angle,
elbow_angle, u_applied, exitflag)`. The command clamp `command_umax` is the runtime-settable
`TuningGains` field; the MPC's weights and structure are set here, with Dyad's `__`-separated
override paths, e.g. `control_system__Q1 = diagm([100, 100, 1, 1])` or `umax = 8.0`.

`Ts` is both the clock period and the MPC's shooting interval, `Np` the horizon in intervals.
`dynamics` is the prediction model; the default is the identified `QubePendulum`.
"""
function generate_mpc_controller(; Ts = 0.01, Np = 60, log_file = MPC_LOG_FILE,
                                  dynamics = furuta_mpc_dynamics(), param_overrides = nothing,
                                  overrides...)
    # The MPC's `u` is an array variable of the clocked partition. Registry SynchToolkit 0.5.0
    # indexes its clock table by the array element and fails inside `stkcompile` with
    # `KeyError: key (control_system₊mpc₊u(t))[1] not found`; the branch MPCComponents pins
    # (JuliaComputing/SynchToolkit.jl#185) looks the element up through `lookup_var_clock`.
    isdefined(SynchToolkit, :lookup_var_clock) ||
        error("this SynchToolkit ($(pkgversion(SynchToolkit)) at $(pkgdir(SynchToolkit))) cannot \
               compile a program with array clocked variables, which the MPC's outputs are. \
               Resolve SynchToolkit from the mpccomponents/compat-synchjulia-0.6 branch -- the \
               [sources] of this package's Project.toml pin it and the rest of the MPC stack; \
               see the README's \"Nonlinear MPC\" section.")
    return compile_program(FurutaMPCHardware; name = :mpc_controller, Ts, Np, dynamics,
                           tunables = MPC_TUNABLES, outputs = _mpc_outputs,
                           log = mpc_log(log_file), param_overrides, overrides...)
end

"""
    MPCController(; Ts=0.01, Np=60, backend=:julia, log_file=MPC_LOG_FILE, command_umax=nothing, overrides...)

A ready-to-call runtime wrapper around the generated MPC controller, the counterpart of
[`SwingupController`](@ref). Compiles the controller, builds a `SynchExecutable` and populates
the parameter structs.

Advance one control step with `out = controller()`, which reads the encoders, solves the MPC,
writes the motor voltage, logs a row and returns `(; row, shoulder, elbow, u, exitflag)`. Point
it at a device with [`open_hardware!`](@ref) or at a simulator with [`bind_hardware!`](@ref)
first; [`run_program!`](@ref) does the opening, the timing and the closing for a real run.

Only `backend = :julia` is available: the AD Jacobian backend the multibody prediction model
needs has no symbolic form for SynchCompiler to render. `command_umax` (the clamp on the
command before the amplifier) overrides the model's value at instantiation; the rest of the
model, the MPC included, is set with `overrides` at compile time (see
[`generate_mpc_controller`](@ref)).
"""
function MPCController(; Ts = 0.01, Np = 60, backend::Symbol = :julia,
                        log_file = MPC_LOG_FILE, command_umax = nothing, kwargs...)
    backend === :julia ||
        throw(ArgumentError("MPCController runs on the :julia backend only: the multibody \
                             prediction model needs the AD Jacobian backend, which cannot be \
                             exported to C"))
    gen = generate_mpc_controller(; Ts, Np, log_file, kwargs...)
    return make_runtime(gen, MPC_OUTPUT_NAMES; backend, gains = (; command_umax))
end
