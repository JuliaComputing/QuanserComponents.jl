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
it, with one real-time iteration per 10 ms tick over a horizon of 60 samples, the motor voltage
bounded to ±10 V and the arm angle and the velocities bounded softly. The prediction model is the
multibody `QubePendulum` itself: `furuta_mpc_dynamics()` compiles it with `multibody` and hands
it to `continuous_dynamics` with the `ForwardDiff` Jacobian backend, which differentiates a
numeric evaluation of the model. That backend exists because a multibody model's compiled form
contains cached linear solves that the default symbolic Jacobian cannot reconstruct; the same
fact rules out C export, so this controller runs on SynchJulia's Julia backend only.

One controller does both jobs through its weighting. Within 0.3 rad of upright the stage cost is
`design_lqr`'s (`Q1 = diag(1000, 10, 1, 1)`, `Q2 = 100`) and the terminal cost is that design's
LQR cost-to-go (`furuta_mpc_terminal_weight`), so the balancing is the well-tried LQR, constraints
aside. Beyond 0.8 rad the stage cost is a swing-up weighting (`Q1_swing = diag(10, 300, 1, 1)`,
`Q2_swing = 10`) that makes pumping the pendulum up pay off within the horizon; in between the two
are blended (`SwingupBlend`, `ACADOSMPC`'s `blend_weights`). Three further details were found
necessary by simulating the loop with quantized angles and the discrete velocity estimators, and
are documented on the components: the MPC is fed the continuous encoder angle with the reference
at the nearest upright (`NearestUpright`) rather than a wrapped angle; soft velocity bounds and
`ACADOSMPC`'s `reset_on_failure` keep a shifted real-time iteration from derailing; and the
velocity estimate must be nearly unfiltered (`velocity_filter = 0.8`; the old default of 0.5 makes
even the balancing unstable at 10 ms). HPIPM condenses the QP to 5 stages, which halves the
worst-case solve time.

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

`test/hardware_mpc.jl` runs it on the rig. Four parameters are runtime-settable (`TuningGains`):
`command_umax`, a clamp on the command before the amplifier for a first run at reduced voltage,
`velocity_filter`, and the blend angles `blend_lower`/`blend_upper`. The script then runs the
same model against the device a second way, as a simulation (`run_mpc_hardware_model`:
`HardwareDiagnostics(realtime = true)` paces the ODE solver's ticks on the wall clock,
`output_trajectories = true` makes the MPC record its predictions) and opens
`MPCComponents.mpc_gui` on the solution to inspect the predicted trajectories and the solver
residuals tick by tick.

`test/mpc_rollouts.jl` ticks the compiled hardware program -- the very node the rig runs --
against a simulated pendulum (the multibody model with encoder quantization, RK4 at five
sub-steps per period, optionally with perturbed parameters) from random initial conditions (arm
within ±1.5 rad, pendulum anywhere, arm velocity within ±3 rad/s, pendulum velocity within
±10 rad/s) and counts the rollouts in which the pendulum stays within 0.1 rad of upright for the
last second of 10 s. MONTECARLO_RESULTS

### Environment

MPCComponents is not registered, and the AD Jacobian backend lives on its
`feat/acados-ad-jacobian-backend` branch, which pins branch builds of its own dependencies:
SynchJulia/SynchCompiler 0.6, a SynchToolkit that supports array clocked variables in
`stkcompile` (JuliaComputing/SynchToolkit.jl#185), a DiscreteComponents branch, a LinearMPC
fork and unregistered acados JLLs. **Registry SynchToolkit 0.5.0 does not have the array
support**: with it, compiling `FurutaMPCHardware` fails inside `stkcompile` with

```
KeyError: key (control_system₊mpc₊u(t))[1] not found
```

(`MPCController` checks for this and says so). This branch therefore checks in `Manifest.toml`
and `test/Manifest.toml`, resolved against the pinned stack, with the two packages that have to
come from local checkouts recorded relative to this repository: `../MPCComponents` (at
`feat/acados-ad-jacobian-backend`) and `../MultibodyComponents` (the `~/.julia/dev` checkout;
the registered release does not resolve against these pins). With both next to the repo,

```
julia --project=path/to/QuanserComponents -e 'using Pkg; Pkg.instantiate()'   # the package
julia --project=path/to/QuanserComponents/test test/hardware_mpc.jl           # the scripts
```

is all it takes; check with `using SynchToolkit; isdefined(SynchToolkit, :lookup_var_clock)`.
For an environment of your own, copy the `[sources]` block (and the `[extras]` entries they
refer to) from Project.toml into it. The first `using` precompiles the multibody and acados
trees, a few minutes.

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
