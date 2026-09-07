# The MPC controller as a synchronous program: what is specific to it.
#
# `FurutaMPCHardware` (dyad/furuta_mpc.dyad) is `FurutaHardware` with the swing-up state
# machine replaced by `FurutaMPC`: an `MPCComponents.ACADOSMPC` that swings the pendulum up and
# balances it, solving a constrained nonlinear optimal control problem over the multibody
# `QubePendulum` model every tick. Everything about compiling it, instantiating it and ticking
# it against the rig is shared with the other programs and lives in program.jl; this file
# supplies the log layout, the tunables, the outputs and the one step of building the
# prediction model that is not itself a Dyad component -- the model is
# `FurutaPredictionModel`, and all that happens here is compiling it.
#
# The prediction model is the point of the exercise. `ACADOSMPC` normally rebuilds the
# model's Jacobian symbolically, which a `multibody`-compiled model defeats: its right-hand
# side references cached linear solves (MTK `__diffcacheₘₜₖ` parameters) that
# `Symbolics.build_function` cannot reconstruct. The `ForwardDiff` Jacobian backend instead
# evaluates the model numerically through MTK's `generate_control_function` -- whose caches
# are `PreallocationTools.DiffCache`s, hence dual-number safe -- and differentiates that. So
# the very plant model serves as the prediction model, written as a Dyad component like any
# other, with no hand-written equations and no DAE reformulation. The price is that there is
# no symbolic form to render to C: this controller runs on SynchJulia's Julia backend only.

# `using`, not `import`, and deliberately un-narrowed: the Dyad-generated components call
# MPCComponents' exported helpers *unqualified* (`diagonal`, in `FurutaMPC`'s `Q1` default),
# and `generated/definitions.jl` only does `import MPCComponents`, which binds the module
# name and nothing in it. Narrowing this to the names used below leaves `diagonal`
# unresolvable in this module and `FurutaMPC` fails to build.
using MPCComponents
using MPCComponents: continuous_dynamics, ContinuousDynamics

export furuta_mpc_dynamics, generate_mpc_controller, MPCController, mpc_log,
       run_mpc_hardware_model

# ---------------------------------------------------------------------------
## The prediction model
# ---------------------------------------------------------------------------
"""
    furuta_mpc_dynamics(; idparams = identified, jacobian_backend = :forwarddiff) -> ContinuousDynamics

The prediction model of `FurutaMPC`: `FurutaPredictionModel` with parameter set `idparams`,
compiled with `MultibodyComponents.multibody` (input: the motor voltage) and wrapped by
`MPCComponents.continuous_dynamics` with an AD Jacobian backend.

The model itself is Dyad's -- the `QubePendulum` plus the `pendulum_energy_ratio` signal the
swing-up term weights -- so what happens here is only the compilation, which is the part no
component can express. See `FurutaPredictionModel` (dyad/furuta_mpc.dyad) for the equations.

The result is a pure ODE with the four states `FURUTA_MPC_STATES` -- the joint angles
`shoulder_joint.phi`, `elbow_joint.phi` and their derivatives -- and the motor voltage as
its one input, in `qube₊`-prefixed signal names, plus `pendulum_energy_ratio`.

`jacobian_backend` must be an AD backend (`:forwarddiff` or `:finitediff`): the compiled
multibody model references cached linear solves that the `:symbolic` backend cannot
reconstruct. That is rejected here rather than after the model has been built, since
building it is the expensive half. The model is built once per argument combination and
cached, so constructing several controllers -- or the simulation model next to the hardware
program -- does not recompile it.
"""
function furuta_mpc_dynamics(; idparams = identified, jacobian_backend::Symbol = :forwarddiff)
    jacobian_backend in (:forwarddiff, :finitediff) ||
        throw(ArgumentError("furuta_mpc_dynamics needs an AD Jacobian backend (:forwarddiff \
                             or :finitediff), got $(repr(jacobian_backend)): the compiled \
                             multibody model references cached linear solves that the \
                             :symbolic backend cannot reconstruct"))
    return get!(_MPC_DYNAMICS_CACHE, (idparams, jacobian_backend)) do
        @info "Compiling the Furuta prediction model for the MPC" jacobian_backend
        model = FurutaPredictionModel(; name = :furuta, idparams)
        # `qube.voltage` rather than a port of the wrapper: the input has to be a variable of
        # the flattened system, and an added port would only be aliased onto this one. It is
        # reached through the un-namespaced view for the same reason `compile_program` does
        # that -- the root's own name is not part of the flattened symbol names, so this is
        # `qube₊voltage`, spelled the way `FURUTA_MPC_STATES` spells the states.
        nsys = ModelingToolkit.toggle_namespacing(model, false)
        u = [nsys.qube.voltage]
        ssys = MultibodyComponents.multibody(model; inputs = u)
        continuous_dynamics(ssys; inputs = u, jacobian_backend)
    end
end
const _MPC_DYNAMICS_CACHE = Dict{Any, ContinuousDynamics}()

# ---------------------------------------------------------------------------
## The program
# ---------------------------------------------------------------------------
"The log `FurutaMPCHardware` writes, in `MPC_LOG_COLUMNS` order. `file` defaults to `MPC_LOG_FILE`."
mpc_log(file = MPC_LOG_FILE) = ProgramLog(file, MPC_LOG_COLUMNS)

# The runtime-settable parameters: the command clamp before the amplifier and the velocity filter
# constant, root parameters of `FurutaMPCHardware` bound down with `final` (see `resolve_tunables`
# for why the root is the right place). The MPC's weights and constraints are structural and are
# set with `overrides` (`control_system__energy_weight = 3e4`).
const MPC_TUNABLES = OrderedDict{Any, Symbol}(
    (nsys -> nsys.command_umax) => :command_umax,
    (nsys -> nsys.velocity_filter) => :velocity_filter,
)

# Node outputs, in the order the runtime reports them: the swing-up program's four, then
# acados' status. The four are the same signals of the same components, so they are taken
# from there rather than restated.
_mpc_outputs(nsys) = [_swingup_outputs(nsys); nsys.control_system.exitflag]
const MPC_OUTPUT_NAMES = (:row, :shoulder, :elbow, :u, :exitflag)

"""
    generate_mpc_controller(; Ts=0.01, Np=60, log_file=MPC_LOG_FILE, overrides...)

Compile the MPC controller to a SynchJulia node: build `FurutaMPCHardware` -- the `ACADOSMPC`
swing-up and balancing controller wired between `HardwareMeasurement` and `HardwareCommand`,
with a `DataLogger`, on a `PeriodicClock` at sample time `Ts` -- and `stkcompile` it.

Returns what [`compile_program`](@ref) returns. The node's argument order is
`(tick::Bool, gains::TuningGains, auto::AutoPars)` and the outputs are `(row, shoulder_angle,
elbow_angle, u_applied, exitflag)`. The command clamp `command_umax` and the velocity filter
constant are the runtime-settable `TuningGains` fields; the MPC's weights and structure are set
here, with Dyad's `__`-separated override paths, e.g. `control_system__energy_weight = 3e4` or
`umax = 8.0`.

`Ts` is both the clock period and the MPC's shooting interval, `Np` the horizon in intervals.
The prediction model is `FurutaMPCHardware`'s own default, `furuta_mpc_dynamics()`; pass
`dynamics = ...` among the `overrides` to predict with a different plant.
"""
function generate_mpc_controller(; Ts = 0.01, Np = 60, log_file = MPC_LOG_FILE,
                                  param_overrides = nothing, overrides...)
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
    return compile_program(FurutaMPCHardware; name = :mpc_controller, Ts, Np,
                           tunables = MPC_TUNABLES, outputs = _mpc_outputs,
                           log = mpc_log(log_file), param_overrides, overrides...)
end

"""
    MPCController(; Ts=0.01, Np=60, backend=:julia, log_file=MPC_LOG_FILE, command_umax=nothing, velocity_filter=nothing, overrides...)

A ready-to-call runtime wrapper around the generated MPC controller, the counterpart of
[`SwingupController`](@ref). Compiles the controller, builds a `SynchExecutable` and populates
the parameter structs.

Advance one control step with `out = controller()`, which reads the encoders, solves the MPC,
writes the motor voltage, logs a row and returns `(; row, shoulder, elbow, u, exitflag)`. Point
it at a device with [`open_hardware!`](@ref) or at a simulator with [`bind_hardware!`](@ref)
first; [`run_program!`](@ref) does the opening, the timing and the closing for a real run.

Only `backend = :julia` is available: the AD Jacobian backend the multibody prediction model
needs has no symbolic form for SynchCompiler to render. `command_umax` (the clamp on the
command before the amplifier) and `velocity_filter` override the model's values at
instantiation; the rest of the model, the MPC included, is set with `overrides` at
compile time (see [`generate_mpc_controller`](@ref)).
"""
function MPCController(; Ts = 0.01, Np = 60, backend::Symbol = :julia,
                        log_file = MPC_LOG_FILE, command_umax = nothing, velocity_filter = nothing,
                        kwargs...)
    backend === :julia ||
        throw(ArgumentError("MPCController runs on the :julia backend only: the multibody \
                             prediction model needs the AD Jacobian backend, which cannot be \
                             exported to C"))
    gen = generate_mpc_controller(; Ts, Np, log_file, kwargs...)
    return make_runtime(gen, MPC_OUTPUT_NAMES; backend, gains = (; command_umax, velocity_filter))
end

# ---------------------------------------------------------------------------
## The hardware model as a simulation, for the MPC debug GUI
# ---------------------------------------------------------------------------
"""
    run_mpc_hardware_model(; Tf, Ts=0.01, Np=60, arm_deg=0.0, card_options=nothing, mode=:hil, log_file=MPC_LOG_FILE, warmup=0.2, disable_gc=true, overrides...)

Run `FurutaMPCHardware` against the device as a *simulation* and return `(; model, sol)`, ready
for `MPCComponents.mpc_gui(model, sol)`.

The program route ([`MPCController`](@ref) + [`run_program!`](@ref)) compiles the model to a
synchronous node that keeps nothing but its outputs; the MPC's predicted trajectories and
solver residuals, which `mpc_gui` shows, are clocked variables that only a solution object
records. So this builds the model with `output_trajectories = true`, compiles it with
`compile_lustre` and lets an ODE solver step the clocked partition -- with `realtime = true`,
so `HardwareDiagnostics` paces the ticks on the wall clock instead of letting the solver run
ahead (the hardware I/O happens inside the ticks exactly as in the program). Warm, a tick
costs what the program's does; cold, the first solve compiles for tens of seconds, which
would leave every tick of a paced run behind schedule. So the problem is first solved for
`warmup` seconds with the command clamped to 0 V (`command_umax = 0`: the encoders are read,
nothing is written to the motor), the timing is reset, and only then is the real run made
and logged. Opening and closing the device and the log around all of that, and switching the
garbage collector off for the run, is [`with_rig`](@ref)'s job, the same as for
[`run_program!`](@ref) -- including `disable_gc`, which is worth setting to `false` for a
long run here, since the acados callbacks allocate a few MB per tick.
`mode = :callback` runs against whatever [`bind_hardware!`](@ref) installed.

`overrides` are Dyad `__`-paths into the model, e.g.
`control_system__Q2 = diagm([1e5])`. The solution's `diagnostics.late` says by how much each
tick overran its slot.
"""
function run_mpc_hardware_model(; Tf, Ts = 0.01, Np = 60, arm_deg = 0.0,
                                 card_options::Union{Nothing, AbstractString} = nothing,
                                 mode::Symbol = :hil, log_file = MPC_LOG_FILE, warmup = 0.2,
                                 disable_gc::Bool = true, overrides...)
    ensure_qube_hw()
    ensure_qube_log()
    model = FurutaMPCHardware(; name = :mpc_hardware, Ts, Np, log_file, realtime = true,
                              output_trajectories = true, overrides...)
    ssys = mtkcompile(model; additional_passes = [SynchToolkit.compile_lustre])
    # The model is purely discrete (no continuous unknowns), and ModelingToolkit's initialization
    # problem cannot be built for its array-valued clocked variables; nothing needs initializing
    # here, so it is skipped.
    prob = ODEProblem(ssys, Pair[], (0.0, Float64(Tf)); build_initializeprob = false)
    # Compile everything the run touches while the motor command is clamped to 0 V -- the
    # encoders are read, nothing is written -- and before the log is open, so the warm-up's
    # ticks are neither logged nor timed. `with_rig` resets the counters and the timing after
    # it, so the run that follows starts from zero.
    function warm()
        wp = ODEProblem(ssys, Pair[ssys.command_umax => 0.0], (0.0, Float64(warmup));
                        build_initializeprob = false)
        solve(wp; dt = Ts)
        return nothing
    end
    sol = with_rig(; mode, arm_deg, card_options, log = mpc_log(log_file), disable_gc,
                    prepare = warmup > 0 ? warm : nothing) do
        solve(prob; dt = Ts)
    end
    return (; model, sol)
end
