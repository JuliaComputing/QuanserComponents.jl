# The MPC controller as a synchronous program: what is specific to it.
#
# `FurutaMPCHardware` (dyad/furuta_mpc.dyad) is `FurutaHardware` with the swing-up state
# machine replaced by `FurutaMPC`: an `MPCComponents.ACADOSMPC` that swings the pendulum up and
# balances it, solving a constrained nonlinear optimal control problem over the multibody
# `QubePendulum` model every tick. `FurutaMPCMultirateHardware` is the same controller with the
# encoders read and the state estimated on a faster clock than the MPC solves on. Everything
# about compiling either, building its parameter structs and ticking it against the rig is
# shared with the other programs and lives in program.jl; this file supplies the two
# `ProgramSpec`s and the one step of building the prediction model that is not itself a Dyad
# component -- the model is `FurutaPredictionModel`, and all that happens here is compiling it.
#
#     gen  = compile_program(FurutaMPCHardware; Ts = 0.01, Np = 60)
#     ctrl = ProgramRuntime(gen; command_umax = 5.0)
#     run_inprocess!(ctrl; Tf = 10)
#     run_ode!(FurutaMPCHardware, prob)     # `prob` over the model built with `spec.ode_kwargs`
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

export furuta_mpc_dynamics, mpc_log

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

`jacobian_backend` must be an AD backend (`:forwarddiff`, `:finitediff` or their `:sparse_`
variants): the compiled multibody model references cached linear solves that the `:symbolic`
backend cannot reconstruct. That is rejected here rather than after the model has been built,
since building it is the expensive half. The backend is chosen here and nowhere else --
`ACADOSMPC` takes it from the `ContinuousDynamics` it is handed.

`:forwarddiff` is the one to use. `:finitediff` allocates 70 % more per tick for a worse tail and
an approximate Jacobian, and the two `:sparse_` backends cannot be built for this model: sparsity
detection traces the right-hand side with SparseConnectivityTracer, whose global tracer carries no
value, and the pivot search of the linear solve inside the compiled multibody model throws on it.
They would not pay off if they could -- the last two rows of this Jacobian are dense, so a column
colouring needs one direction per entry of `[x; u]`, exactly what the dense backend does. They are
accepted here anyway, so that the day detection works the comparison is one keyword away. See
NOTES.md.

The model is built once per argument combination and cached, so constructing several controllers
-- or the simulation model next to the hardware program -- does not recompile it.
"""
function furuta_mpc_dynamics(; idparams = identified, jacobian_backend::Symbol = :forwarddiff)
    jacobian_backend in (:forwarddiff, :finitediff, :sparse_forwarddiff, :sparse_finitediff) ||
        throw(ArgumentError("furuta_mpc_dynamics needs an AD Jacobian backend (:forwarddiff, \
                             :finitediff, :sparse_forwarddiff or :sparse_finitediff), got \
                             $(repr(jacobian_backend)): the compiled multibody model \
                             references cached linear solves that the :symbolic backend \
                             cannot reconstruct"))
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
## The programs
# ---------------------------------------------------------------------------
"The log `FurutaMPCHardware` writes, in `MPC_LOG_COLUMNS` order. `file` defaults to `MPC_LOG_FILE`."
mpc_log(file = MPC_LOG_FILE) = ProgramLog(file, MPC_LOG_COLUMNS)

# Node outputs, in the order the runtime reports them: the swing-up program's four, then
# acados' status. The four are the same signals of the same components, so they are taken
# from there rather than restated.
_mpc_outputs(nsys) = [_swingup_outputs(nsys); nsys.control_system.exitflag]
const MPC_OUTPUT_NAMES = (:row, :shoulder, :elbow, :u, :exitflag)

# The MPC's `u` is an array variable of the clocked partition. Registry SynchToolkit 0.5.0
# indexes its clock table by the array element and fails inside `stkcompile` with
# `KeyError: key (control_system₊mpc₊u(t))[1] not found`; the branch this package pins
# (JuliaComputing/SynchToolkit.jl#185) looks the element up through `lookup_var_clock`, so
# that function's presence is the feature test. SynchToolkit `main` would fail it: it
# dropped `lookup_var_clock` with the #186 clock rework and never re-landed #185, which is
# why the pin is a branch and not `main` even though `main` has the `Latest` operator the
# multirate model needs.
function _mpc_prerequisites()
    isdefined(SynchToolkit, :lookup_var_clock) ||
        error("this SynchToolkit ($(pkgversion(SynchToolkit)) at $(pkgdir(SynchToolkit))) cannot \
               compile a program with array clocked variables, which the MPC's outputs are. \
               Resolve SynchToolkit from the mpccomponents/sj0.8 branch -- the [sources] of this \
               package's Project.toml pin it and the rest of the MPC stack; see the README's \
               \"Nonlinear MPC\" section.")
    return
end

function _mpc_multirate_prerequisites()
    _mpc_prerequisites()
    isdefined(SynchToolkit, :Latest) ||
        error("this SynchToolkit ($(pkgversion(SynchToolkit)) at $(pkgdir(SynchToolkit))) has no \
               `Latest` operator, which the multirate model's clock transitions need. Resolve it \
               from the mpccomponents/sj0.8 branch -- the [sources] of this package's Project.toml \
               pin it; see the README's \"Nonlinear MPC\" section.")
    return
end

# What the model has to be built with for `run_ode!`: `realtime = true` makes `HardwareDiagnostics` pace the
# ticks on the wall clock, and `output_trajectories = true` makes the MPC record its predicted
# trajectories and solver residuals, which is what that route is for. Its warm-up clamps the
# command to 0 V through the root `command_umax`, so the encoders are read and nothing is written
# while the first solve compiles.
const _MPC_ODE_KWARGS = (; realtime = true, output_trajectories = true)
_mpc_ode_warmup(ssys) = Pair[ssys.command_umax => 0.0]

# The runtime-settable parameters are the command clamp before the amplifier -- for a first run
# at less voltage than the MPC is allowed to plan with -- and the velocity estimator's constant,
# root parameters of `FurutaMPCHardware` bound down with `final` (see `resolve_tunables` for why
# the root is the right place). The MPC's weights and constraints are structural and are set with
# `overrides` to `compile_program` (`control_system__energy_weight = 3e4`, `umax = 8.0`); `Ts`
# is both the clock period and the MPC's shooting interval, `Np` the horizon in intervals, and
# `dynamics = ...` replaces the prediction model `furuta_mpc_dynamics()`.
#
# Only the `:julia` backend: the AD Jacobian backend the multibody prediction model needs has no
# symbolic form for SynchJulia to render to C.
program_spec(::typeof(FurutaMPCHardware)) = ProgramSpec(;
    name = :mpc_controller,
    tunables = OrderedDict{Any, Symbol}((nsys -> nsys.command_umax) => :command_umax,
                                        (nsys -> nsys.velocity_filter) => :velocity_filter),
    outputs = _mpc_outputs,
    output_names = MPC_OUTPUT_NAMES,
    log = mpc_log,
    backends = (:julia,),
    prerequisites = _mpc_prerequisites,
    ode_kwargs = _MPC_ODE_KWARGS,
    ode_warmup = _mpc_ode_warmup)

# The same for `FurutaMPCMultirateHardware`, whose estimator is an alpha-beta-gamma tracker
# rather than a difference through an exponential filter: `velocity_alpha` is its position
# correction gain, and the rate and acceleration gains follow from it, so there is still one
# runtime-settable parameter for the estimation. The model declares two clocks, `Ts_fast` for
# the measurement and `Ts` for the MPC; `compile_program` reads them off the model, so the
# compiled node takes one boolean per clock and the runtime ticks the fast one. `Ts` has to be an
# integer multiple of `Ts_fast`, and a power-of-two multiple keeps the two clocks ticking at the
# same instants without floating-point drift.
program_spec(::typeof(FurutaMPCMultirateHardware)) = ProgramSpec(;
    name = :mpc_multirate_controller,
    tunables = OrderedDict{Any, Symbol}((nsys -> nsys.command_umax) => :command_umax,
                                        (nsys -> nsys.velocity_alpha) => :velocity_alpha),
    outputs = _mpc_outputs,
    output_names = MPC_OUTPUT_NAMES,
    log = mpc_log,
    backends = (:julia,),
    prerequisites = _mpc_multirate_prerequisites,
    ode_kwargs = _MPC_ODE_KWARGS,
    ode_warmup = _mpc_ode_warmup)
