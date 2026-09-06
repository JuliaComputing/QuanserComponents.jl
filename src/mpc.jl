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
import ControlSystemsBase
using ControlSystemsBase: c2d, ss
using LinearAlgebra: I

export furuta_mpc_dynamics, furuta_mpc_terminal_weight, generate_mpc_controller, MPCController,
       mpc_log, run_mpc_hardware_model

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

"""
    furuta_mpc_terminal_weight(dyn, Ts, Q1, Q2) -> P::Matrix

The terminal weight of `FurutaMPC`: the infinite-horizon cost-to-go of the discrete-time LQR
problem with weights `Q1`, `Q2` for the prediction model `dyn` linearized about upright and
discretized at `Ts` -- what `design_lqr` solves for its gain. As the MPC's `Qf` (on the outputs,
which are the states) it is centred on the reference, so it follows the reference to the nearest
upright, unlike `ACADOSMPC`'s `terminal_lqr_cost`, which is centred on a fixed operating point.
"""
function furuta_mpc_terminal_weight(dyn::ContinuousDynamics, Ts, Q1, Q2)
    nx, nu = dyn.nx, dyn.nu
    J = zeros(nx, nx + nu)
    dyn.J!(J, [0.0, pi, 0.0, 0.0], zeros(nu), dyn.p_default, 0.0)
    sysd = c2d(ss(J[:, 1:nx], J[:, nx+1:end], Matrix{Float64}(I, nx, nx), zeros(nx, nu)), Ts)
    P = ControlSystemsBase.are(ControlSystemsBase.Discrete, Matrix(sysd.A), Matrix(sysd.B),
                               Matrix{Float64}(Q1), Matrix{Float64}(Q2))
    return Matrix{Float64}((P + P') / 2)
end

# ---------------------------------------------------------------------------
## The program
# ---------------------------------------------------------------------------
"The log `FurutaMPCHardware` writes, in `MPC_LOG_COLUMNS` order. `file` defaults to `MPC_LOG_FILE`."
mpc_log(file = MPC_LOG_FILE) = ProgramLog(file, MPC_LOG_COLUMNS)

# The runtime-settable parameters: the command clamp before the amplifier, the velocity filter
# constant and the two blend angles, root parameters of `FurutaMPCHardware` bound down with
# `final` (see `resolve_tunables` for why the root is the right place). The MPC's weights are
# structural (the terminal cost-to-go is computed from them at build time) and are set with
# `overrides` (`control_system__Q1_swing = ...`).
const MPC_TUNABLES = OrderedDict{Any, Symbol}(
    (nsys -> nsys.command_umax) => :command_umax,
    (nsys -> nsys.velocity_filter) => :velocity_filter,
    (nsys -> nsys.blend_lower) => :blend_lower,
    (nsys -> nsys.blend_upper) => :blend_upper,
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
elbow_angle, u_applied, exitflag)`. The command clamp `command_umax`, the velocity filter constant
and the blend angles are the runtime-settable `TuningGains` fields; the MPC's weights and
structure are set here, with Dyad's `__`-separated override paths, e.g.
`control_system__Q1_swing = diagm([10, 300, 1, 1])` or `umax = 8.0`.

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
    MPCController(; Ts=0.01, Np=60, backend=:julia, log_file=MPC_LOG_FILE, command_umax=nothing, velocity_filter=nothing, blend_lower=nothing, blend_upper=nothing, overrides...)

A ready-to-call runtime wrapper around the generated MPC controller, the counterpart of
[`SwingupController`](@ref). Compiles the controller, builds a `SynchExecutable` and populates
the parameter structs.

Advance one control step with `out = controller()`, which reads the encoders, solves the MPC,
writes the motor voltage, logs a row and returns `(; row, shoulder, elbow, u, exitflag)`. Point
it at a device with [`open_hardware!`](@ref) or at a simulator with [`bind_hardware!`](@ref)
first; [`run_program!`](@ref) does the opening, the timing and the closing for a real run.

Only `backend = :julia` is available: the AD Jacobian backend the multibody prediction model
needs has no symbolic form for SynchCompiler to render. `command_umax` (the clamp on the
command before the amplifier), `velocity_filter` and the blend angles override the model's
values at instantiation; the rest of the model, the MPC included, is set with `overrides` at
compile time (see [`generate_mpc_controller`](@ref)).
"""
function MPCController(; Ts = 0.01, Np = 60, backend::Symbol = :julia,
                        log_file = MPC_LOG_FILE, command_umax = nothing, velocity_filter = nothing,
                        blend_lower = nothing, blend_upper = nothing, kwargs...)
    backend === :julia ||
        throw(ArgumentError("MPCController runs on the :julia backend only: the multibody \
                             prediction model needs the AD Jacobian backend, which cannot be \
                             exported to C"))
    gen = generate_mpc_controller(; Ts, Np, log_file, kwargs...)
    return make_runtime(gen, MPC_OUTPUT_NAMES; backend,
                        gains = (; command_umax, velocity_filter, blend_lower, blend_upper))
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
and logged. The device and the log are opened and closed around it as `run_program!` does.
`mode = :callback` runs against whatever [`bind_hardware!`](@ref) installed.

As in [`run_program!`](@ref), the garbage collector is off during the run unless
`disable_gc = false` (the acados callbacks allocate a few MB per tick; a collection pause is
longer than a period). `overrides` are Dyad `__`-paths into the model, e.g.
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
    open_hardware!(mode; arm_deg, card_options)
    sol = try
        if warmup > 0
            # Compile everything the solve touches while the motor command is clamped to zero.
            warm = ODEProblem(ssys, Pair[ssys.command_umax => 0.0], (0.0, Float64(warmup));
                              build_initializeprob = false)
            solve(warm; dt = Ts)
        end
        reset_hardware_counters!()
        open_log!(mpc_log(log_file))
        GC.gc()
        disable_gc && GC.enable(false)
        solve(prob; dt = Ts)
    finally
        GC.enable(true)
        close_hardware!()
        close_log!()
    end
    return (; model, sol)
end
