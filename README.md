# QuanserComponents

Dyad models of the Quanser QUBE-Servo 2 with the rotary pendulum attachment (a Furuta
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

## Layout

```
dyad/        the Dyad models: plant, controller, hardware I/O, analyses
generated/   Julia code generated from dyad/ (checked in)
src/         hand-written Julia: hardware operators, program compilation, C export, deployment
csrc/        the C side: Quanser HIL I/O, logging, trajectory replay, the run_hardware harness
assets/      component icons, QUBE meshes and textures
examples/    parameter identification scripts (their own environment, see examples/Project.toml)
test/        test suite (its own environment, see test/Project.toml)
```
