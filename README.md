# QuanserComponents

Dyad models of the Quanser QUBE-Servo 3 with the rotary pendulum attachment (a Furuta
pendulum): a multibody plant, a clocked swing-up and balancing controller, and the
components that let the same controller run on the real device as a synchronous program.

https://github.com/user-attachments/assets/5ebe76e0-c05f-45de-a816-7d8e877a1d93

## Installation

The package and its Dyad dependencies are registered in the
[DyadRegistry](https://github.com/JuliaComputing/DyadRegistry); everything else comes from
the General registry. The generated Julia code is checked in, so the Dyad compiler is not
needed to use the package.

```julia
pkg> registry add https://github.com/JuliaComputing/DyadRegistry
pkg> add https://github.com/JuliaComputing/QuanserComponents.jl
```

or, for development, clone the repository and `pkg> dev path/to/QuanserComponents`.

## Simulating the swing-up

`FurutaSwingup` is the closed loop: the multibody `QubePendulum` plant, the discrete-time
`SwingupWithHoming` controller on a 5 ms clock, samplers and a zero-order hold. The controller
is a clocked partition of the model; `SynchToolkit.compile_lustre` compiles it beside the
continuous plant so the ODE solver can step the two together.

```julia
using QuanserComponents
using ModelingToolkit, MultibodyComponents, SynchToolkit
using OrdinaryDiffEqDefault

@named model = FurutaSwingup()
ssys = multibody(model, additional_passes = [SynchToolkit.compile_lustre])
prob = ODEProblem(ssys, [
    ssys.qubependulum.shoulder_joint.render => false
    ssys.qubependulum.elbow_joint.phi => deg2rad(0.15)   # start almost exactly hanging down
    ssys.qubependulum.shoulder_joint.phi => 0.0
    ssys.gain.k => 1.0                                   # actuator gain seen by the plant
], (0.0, 10.0))
sol = solve(prob; dt = 0.005)

phi = sol[ssys.qubependulum.elbow_joint.phi]
rad2deg(abs(mod2pi(phi[end]) - pi))   # deviation from upright at t = 10 s, about 0.004 degrees
```

The plant uses the `identified` parameter set while the controller was tuned for the
datasheet values, so the first swing-up attempt is not caught: the arm runs out of bounds, the
controller re-homes and the second attempt holds. Useful signals to plot (with Plots.jl or
Makie in your own environment):

```julia
using Plots
plot(sol, idxs = [ssys.qubependulum.elbow_joint.phi,
                  ssys.qubependulum.shoulder_joint.phi,
                  ssys.zeroorderhold.y])                         # angles [rad] and motor voltage [V]
plot(sol, idxs = ssys.control_system.runtime.swingup_catch.neartop.y)   # stabilizer active
```

### 3D rendering

Rendering needs GLMakie in the environment (it is not a dependency of the package):

```julia
import GLMakie
using MultibodyComponents: render
render(model, sol, 0.0)                          # still at t = 0, with a time slider
render(model, sol; filename = "swingup.mp4")     # animation, real time at 30 frames per second
```

## Running on the hardware

`FurutaHardware` is the same controller with the plant replaced by `HardwareMeasurement`,
`HardwareCommand`, `HardwareDiagnostics` and a `DataLogger`. Because it is purely discrete,
`SynchToolkit.stkcompile` turns it into a standalone synchronous program that can run in
process (`SwingupController(; backend = :julia)` or `:c`) or be exported as standalone C
(`export_swingup_c`). The `FurutaSwingupExperiment` analysis builds the program, exports the C,
copies it to a Raspberry Pi with the QUBE attached, builds and runs it there and streams the log
back. Without a device, `FurutaSwingupExperiment(; run = false)` still builds and exports.

The hardware I/O calls into `csrc/qube_hw.c`, which needs the Quanser HIL SDK; the exported C
is built on the target for the same reason. See `test/runtests.jl` for how the controller is
built, stepped and compared across backends without a device attached.

## Nonlinear MPC swing-up (experimental)

`FurutaMPC` (dyad/furuta_mpc.dyad) replaces the whole swing-up state machine by one nonlinear
model-predictive controller, `MPCComponents.ACADOSMPC`: it swings the pendulum up and balances
it, solving every 10 ms a constrained optimal control problem over a horizon of 60 samples with
the motor voltage bounded to ±10 V, the arm bounded to the end stops (a soft state constraint),
the upright state as reference and the infinite-horizon LQR cost-to-go about upright as terminal
cost. The prediction model is the multibody `QubePendulum` itself: `furuta_mpc_dynamics()`
compiles it with `multibody` and hands it to `continuous_dynamics` with the `ForwardDiff`
Jacobian backend, which differentiates a numeric evaluation of the model. That backend exists
because a multibody model's compiled form contains cached linear solves that the default
symbolic Jacobian cannot reconstruct; the same fact rules out C export, so this controller runs
on SynchJulia's Julia backend only.

The velocities the MPC is fed are filtered first differences of the encoder angles, half a sample
behind, so the control penalty must not be small relative to the state weights (a penalty a
thousand times smaller than the defaults destabilized the balancing in earlier tests). The slack
weight of the arm constraint is a moderate 1e3: with acados' default of 1e6 the constraint
dominates the cost whenever the arm is past the limit and the MPC sacrifices the pendulum to
haul the arm back. The measured elbow angle is wrapped to `[0, 2π)`, with the cut at the hanging
position where the model is periodic and the cost symmetric.

`FurutaMPCSwingup` is the closed loop around the simulated plant and `FurutaMPCHardware` the
hardware program, the counterparts of `FurutaSwingup` and `FurutaHardware`:

```julia
using QuanserComponents, ModelingToolkit, MultibodyComponents, SynchToolkit, OrdinaryDiffEqDefault
@named model = FurutaMPCSwingup()
ssys = multibody(model, additional_passes = [SynchToolkit.compile_lustre])
prob = ODEProblem(ssys, Pair[ssys.qubependulum.shoulder_joint.render => false,
                             ssys.qubependulum.elbow_joint.phi => deg2rad(0.15),
                             ssys.qubependulum.shoulder_joint.phi => 0.0], (0.0, 10.0))
sol = solve(prob; dt = 0.01)

ctrl = MPCController(; Ts = 0.01, Np = 60)     # the hardware program, see test/hardware_mpc.jl
```

`test/hardware_mpc.jl` runs it on the rig; `command_umax` clamps the command before the
amplifier and is the one runtime-settable parameter, for a first run at reduced voltage.
`test/mpc_rollouts.jl` ticks the compiled hardware program -- the very node the rig runs --
against a simulated pendulum (the multibody model with encoder quantization, RK4 at five
sub-steps per period, optionally with perturbed parameters) from a thousand random initial
conditions and reports success statistics, catch times and tick timings. The MPC-only
configuration has not been run through it yet; assets/mpc/ holds the results of the earlier
configuration, in which an energy-based swing-up handed over to the MPC for balancing
(978 of 1000 rollouts balanced within 10 s, no failed solves).

### Environment

MPCComponents is not registered, and the AD Jacobian backend lives on its
`feat/acados-ad-jacobian-backend` branch, which pins branch builds of SynchJulia/SynchCompiler
0.6 and SynchToolkit 0.5 in its own Manifest. This package's `[sources]` entry points at that
branch, so `pkg> instantiate` works once the branch is on GitHub; until then, or to work from a
local checkout, assemble an environment by hand with the same pins MPCComponents uses:

```toml
[deps]                      # plus whatever else the scripts need: Plots, Statistics, ...
MPCComponents = "aba2bcdf-b216-4bf2-af71-aceb9999ebd9"
QuanserComponents = "d44921f8-446f-4040-8b54-e37c69fd1e29"
MultibodyComponents = "01883e52-22cd-4538-b14d-b44f958a131d"
DiscreteComponents = "b5590941-51af-4a5a-9bca-5ae7cd448b75"
SynchToolkit = "f500ffa8-9682-42ac-8d34-0ca7926c2e94"
SynchJulia = "a1b2c3d4-5e6f-7a8b-9c0d-e1f2a3b4c5d6"
SynchCompiler = "5d9dccf6-a926-4748-b7e2-6521ccc431d1"

[extras]
acados_jll = "49ddb18e-ca18-5f65-a4dd-7588daaac186"
tera_renderer_jll = "73a839a6-9370-58f6-a727-029f0c62a9a5"
LinearMPC = "82e1c212-e1a2-49d2-b26a-a31d6968e3bd"

[sources]
QuanserComponents = {path = "path/to/QuanserComponents"}
MPCComponents = {path = "path/to/MPCComponents"}      # checked out at feat/acados-ad-jacobian-backend
DiscreteComponents = {url = "https://github.com/JuliaComputing/DiscreteComponents.git", rev = "mpccomponents/with-d-compat"}
SynchJulia = {url = "https://github.com/JuliaComputing/SynchJulia.jl.git", rev = "mpccomponents/ref-ccall-args"}
SynchCompiler = {url = "https://github.com/JuliaComputing/SynchJulia.jl.git", rev = "mpccomponents/ref-ccall-args", subdir = "SynchCompiler"}
SynchToolkit = {url = "https://github.com/JuliaComputing/SynchToolkit.jl.git", rev = "mpccomponents/compat-synchjulia-0.6"}
LinearMPC = {url = "https://github.com/baggepinnen/LinearMPC.jl.git", rev = "feat/disturbance-cross-term"}
acados_jll = {url = "https://github.com/baggepinnen/acados_jll.jl", rev = "main"}
tera_renderer_jll = {url = "https://github.com/baggepinnen/tera_renderer_jll.jl", rev = "main"}
```

MultibodyComponents has to be the `~/.julia/dev` checkout (the registered release does not
resolve against these pins). The first `using` precompiles the multibody and acados trees, a
few minutes.

## Layout

```
dyad/        the Dyad models: plant, controllers (swing-up state machine and MPC), hardware I/O, analyses
generated/   Julia code generated from dyad/ (checked in)
src/         hand-written Julia: hardware operators, program compilation, C export, deployment
csrc/        the C side: Quanser HIL I/O, logging, trajectory replay, the run_hardware harness
assets/      component icons, QUBE meshes and textures
examples/    parameter identification scripts (their own environment, see examples/Project.toml)
test/        test suite (its own environment, see test/Project.toml), the hardware run scripts and the MPC Monte Carlo
```
