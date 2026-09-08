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

The snippets below reach for four of the dependencies by name, so add them to the
environment as well:

```julia
pkg> add ModelingToolkit MultibodyComponents SynchToolkit OrdinaryDiffEqDefault
```

### The simulation-only branch

Those commands resolve `main` only for someone who can reach the repositories the MPC
controller is pinned to: MPCComponents is not registered, and `[sources]` names branch
builds of SynchToolkit, SynchJulia, DiscreteComponents and LinearMPC, two unregistered
acados JLLs and two local checkouts (see [Environment](#environment) below). Everyone
else installs the branch `simulation-only`, which is `main` as of the commit before the
MPC work and resolves entirely to registered versions:

```julia
pkg> registry add https://github.com/JuliaComputing/DyadRegistry
pkg> add https://github.com/JuliaComputing/QuanserComponents.jl#simulation-only
```

`Manifest.toml` and `test/Manifest.toml` are checked in there and contain no repository
revisions, so a clone plus `Pkg.instantiate()` reproduces the versions the branch was
verified against. It carries the multibody plant, the energy swing-up and balancing
controller, the hardware programs and the C export -- everything but `FurutaMPC` and its
closed loops -- and its test suite passes with no device attached.

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
`SynchToolkit.stkcompile` turns it into a standalone synchronous program. Every program in this
package is handled the same way: `compile_program(Model; ...)` compiles it, with everything the
program needs beyond its constructor declared once in the model's `ProgramSpec`, and the result
is put on the rig by one of three functions, each named for how it runs the program:

```julia
gen  = compile_program(FurutaHardware; Ts = 0.005)
ctrl = ProgramRuntime(gen; backend = :c, umax = 5.0)   # :julia or :c; the tunables by name
run_inprocess!(ctrl; Tf = 10)                          # a Julia timing loop ticks the node here
run_c!(gen; Tf = 10, deploy_host = "pi@192.168.1.49")  # standalone C, built and run here or on the Pi
run_ode!(FurutaMPCHardware, prob)                      # an ODE solver steps the model, paced by itself
```

`prob` is an `ODEProblem` over the model built with the spec's `ode_kwargs` (see `run_ode!`).

The `FurutaSwingupExperiment` analysis builds the program, exports the C, copies it to a Raspberry
Pi with the QUBE attached, builds and runs it there and streams the log back (`run_on_target`
picks between the in-process and the C route from the analysis' parameters). Without a device,
`FurutaSwingupExperiment(; run = false)` still builds and exports.

The hardware I/O calls into `csrc/qube_hw.c`, which needs the Quanser HIL SDK; the exported C
is built on the target for the same reason. See `test/runtests.jl` for how the controller is
built, stepped and compared across backends without a device attached.

## Nonlinear MPC swing-up (experimental)

`FurutaMPC` (dyad/furuta_mpc.dyad) replaces the whole swing-up state machine by one nonlinear
model-predictive controller, `MPCComponents.ACADOSMPC`: it swings the pendulum up and balances
it, with one real-time iteration per 10 ms tick over a horizon of 60 samples, the motor voltage
bounded to ±10 V and the arm angle and the velocities bounded softly. The prediction model is the
multibody `QubePendulum` itself: `FurutaPredictionModel` (dyad/furuta_mpc.dyad) is that plant plus
the one signal the swing-up term needs, and `furuta_mpc_dynamics()` (src/mpc.jl) does nothing but
compile it with `multibody` and hand it to `continuous_dynamics` with the `ForwardDiff` Jacobian
backend, which differentiates a numeric evaluation of the model. That backend exists because a multibody model's compiled form
contains cached linear solves that the default symbolic Jacobian cannot reconstruct; the same
fact rules out C export, so this controller runs on SynchJulia's Julia backend only.

The weighting is `design_lqr`'s (`Q1 = diag(1000, 10, 1, 1)`, `Q2 = 100`, terminal cost the LQR
cost-to-go about upright) plus one term that makes it swing up: *energy shaping in the stage
cost*. The prediction model carries the signal `pendulum_energy_ratio` (kinetic energy of the
rotation about the elbow plus the height of the centre of mass, normalized so that rest upright
is 1 and hanging at rest is 0, the quantity the energy swing-up controller pumps -- though not
by the same expression: `Energy` omits the parallel-axis term), and the MPC
weights it as a fifth controlled output with the reference 1 and the weight `energy_weight`
(1e5). The cost is then a nonlinear least squares, which `ACADOSMPC` gained for this (nonlinear
`outputs`, JuliaComputing/MPCComponents.jl#16). From rest the term gives the solver a gradient
towards pumping from the first tick; a soft *terminal set* on the energy, the previous design,
could not while the set was out of reach, and swung up in one to five seconds with a hesitant
first swing. Near upright the term vanishes quartically in the deviation (the energy error is
quadratic in the angle and in the velocity), and the closed loop from kicks up to 0.3 rad or
4 rad/s is identical to the LQR's, to the voltage. What the term does not change is the arm: under
LQR weights the swing-up still throws it past the end stops in 60 % of the rest starts (2.4 rad
median, up to 8.6 rad, see below), the same distribution as with the terminal set. The LQR arm
weight forbids the gentler pumping, a hard arm bound removes the swing-up altogether (the
real-time iteration fails), and a terminal set on the arm angle does not help either.
Three further details were found necessary by simulating the loop with quantized angles and the
discrete velocity estimators, and are documented on the components: the pendulum angle fed to the
LQR part is wrapped to [0, 2π) about the reference π (the 2π jump at the bottom costs a failed
solve that `reset_on_failure` recovers), soft velocity bounds and `ACADOSMPC`'s `reset_on_failure`
keep a shifted real-time iteration from derailing, and the velocity estimate must be nearly
unfiltered (`velocity_filter = 0.8`; the old default of 0.5 makes even the balancing unstable at
10 ms). HPIPM condenses the QP to 5 stages, which is the measured optimum of that setting
(`qp_cond_N`, swept in NOTES.md).

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

ctrl = ProgramRuntime(compile_program(FurutaMPCHardware; Ts = 0.01, Np = 60))   # the hardware program, see test/hardware_mpc.jl
```

`test/hardware_mpc.jl` runs it on the rig. Two parameters are runtime-settable (`TuningGains`):
`command_umax`, a clamp on the command before the amplifier for a first run at reduced voltage,
and `velocity_filter`. The script then runs the same model against the device a second way, as a
simulation (`run_ode!`: `HardwareDiagnostics(realtime = true)` paces the ODE
solver's ticks on the wall clock, `output_trajectories = true` makes the MPC record its
predictions) and opens `MPCComponents.mpc_gui` on the solution to inspect the predicted
trajectories and the solver residuals tick by tick.

`test/mpc_rollouts.jl` ticks the compiled hardware program -- the very node the rig runs --
against a simulated pendulum (the multibody model with encoder quantization, RK4 at five
sub-steps per period, optionally with perturbed parameters) from random initial conditions (arm
within ±1.5 rad, pendulum anywhere, arm velocity within ±3 rad/s, pendulum velocity within
±10 rad/s) or from rest near hanging as the rig starts, and counts the rollouts in which the
pendulum stays within 0.1 rad of upright for the last second of 10 s. The summaries of four such runs are in assets/mpc/ (the compiled program at the defaults above):

| rollouts | plant | starts | balanced within 10 s | catch time median / 90 % | arm past the stops |
|---|---|---|---|---|---|
| 1000 | identified | random | 1000 | 0.94 s / 1.92 s | 714 (median 2.68 rad, max 12.3) |
| 200 | motor −15 %, arm +20 %, Jp +15 %, damping ×2 | random | 199 | 1.36 s / 3.89 s | 159 (median 3.21 rad, max 12.8) |
| 200 | identified | at rest near hanging, as on the rig | 200 | 1.08 s / 1.82 s | 120 (median 2.38 rad, max 8.58) |
| 200 | randomly perturbed per rollout (±15 % kt, ±20 % arm mass, ±15 % Jp, damping ½ to 2) | at rest near hanging | 200 | 1.08 s / 2.42 s | 128 (median 2.35 rad, max 8.95) |

Solve time was 0.98 to 1.26 ms per tick at the median and 2.2 to 2.6 ms at the 99th percentile;
about one solve in a thousand failed and was retried. Every rest start swings up within 2.5 s
at the 90th percentile and 5.7 s at worst; the one rollout not balanced at 10 s (perturbed plant,
random start) was still swinging. The arm column is the caveat of this design: in 60 % of the
rest starts and 70 % of the random ones the swing-up carries the arm past the ±1.92 rad end stops,
the same distribution the terminal-set design had (its summaries: 195 to 197 of 200 rest starts
balanced, catch median 2.2 s, 90 % 5 s, arm median 2.2 rad).

`qp_cond_N = 5` and the `ForwardDiff` Jacobian backend are both measured optima; the sweep behind
them, and why the sparse AD backends are of no use to this model, are in NOTES.md.

### Multirate: 1 kHz estimation, 125 Hz control

`FurutaMPCMultirate` splits the controller across two clocks -- the encoders read and the state
estimated at `Ts_fast` (1 ms), the MPC solved at `Ts` (8 ms) -- with `FurutaMPCMultirateSwingup`
and `FurutaMPCMultirateHardware` as the simulated and hardware loops. The single-rate models are
unchanged.

Sampling faster is not what makes it better. `VelocityEstimator` is a backward difference through
an exponential filter, which assumes the velocity is locally constant; a swinging pendulum's is
not, so its estimate carries an acceleration-induced bias that no filter tuning removes, and
running it at 1 ms instead of 10 ms measures *worse* while balancing. Estimating the acceleration
as well removes the bias: the two `DiscreteComponents.AlphaBetaGammaFilter`s score 0.035 rad/s RMS
balancing and 0.243 during the swing-up, against 0.141 and 1.280 for the 10 ms baseline. And a
window long enough to do that is only affordable at 1 ms. The measurements, the two metrics that
turned out to be misleading, and the cost of a third-order tracker on a fast disturbance are in
NOTES.md.

The clock transition is `DiscreteComponents.Latest`. It does not *relate* the two clocks: both are
declared separately, and only convention makes one eight times the other. A block cannot derive one
clock from another, so an integer rate relationship that is stated once and checked belongs at the
compiler level instead.

```julia
@named model = FurutaMPCMultirateSwingup(; Ts = 0.008, Ts_fast = 0.001)
ssys = multibody(model, additional_passes = [SynchToolkit.compile_lustre])
sol = solve(ODEProblem(ssys, Pair[ssys.qubependulum.elbow_joint.phi => deg2rad(0.15)],
                       (0.0, 4.0)); dt = 0.001)          # step the *fastest* clock

ctrl = ProgramRuntime(compile_program(FurutaMPCMultirateHardware; Ts = 0.008, Ts_fast = 0.001, Np = 75))
out = ctrl()        # one 1 ms tick; the MPC fires on every 8th, `nothing` in between
```

`Np` is 75 rather than 60 so the horizon stays the 0.6 s that was tuned at 10 ms. `compile_program`
reads the two clocks off the model, the compiled node takes one boolean per clock and
`ProgramRuntime` generates that pattern, so the driver still ticks once per `Ts_fast`; there is no C
harness for it, since `run_hardware.c` drives a single tick.

### Environment

MPCComponents is not registered, and the MPC stack it needs is not all released. The pins that
matter are SynchToolkit's branch `mpccomponents/sj0.8`, which is the only one with both the
`Latest` clock-crossing operator the multirate model needs
(JuliaComputing/SynchToolkit.jl#199) and array clocked variables in `stkcompile` (#185); a
DiscreteComponents integration branch carrying `AlphaBetaGammaFilter`
(JuliaComputing/DiscreteComponents.jl#119) until that merges; and unregistered LinearMPC and
acados forks. SynchJulia is *not* pinned -- 0.8.1 is in DyadRegistry -- and SynchCompiler no
longer exists, having been merged into SynchJulia by JuliaComputing/SynchJulia.jl#205.
**Registry SynchToolkit 0.5.0 does not have the array support**: with it, compiling
`FurutaMPCHardware` fails inside `stkcompile` with

```
KeyError: key (control_system₊mpc₊u(t))[1] not found
```

(`compile_program` checks for this and says so). This branch therefore checks in `Manifest.toml`
and `test/Manifest.toml`, resolved against the pinned stack. MPCComponents comes from its GitHub
`main` (which has to include
[JuliaComputing/MPCComponents.jl#31](https://github.com/JuliaComputing/MPCComponents.jl/pull/31):
without it the multibody prediction model is rejected as time-varying), and one package has to
come from a local checkout recorded relative to this repository: `../MultibodyComponents` (the
`~/.julia/dev` checkout; the registered release does not resolve against these pins). With both
next to the repo,

```
julia --project=path/to/QuanserComponents -e 'using Pkg; Pkg.instantiate()'   # the package
julia --project=path/to/QuanserComponents/test test/hardware_mpc.jl           # the scripts
```

is all it takes; check with `using SynchToolkit` that both
`isdefined(SynchToolkit, :lookup_var_clock)` and `isdefined(SynchToolkit, :Latest)` hold.
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
