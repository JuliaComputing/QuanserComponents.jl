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
# runtime, the in-process loop) and harness.jl (C export, deployment, live plotting). What is
# about it is the `ProgramSpec` below:
#
#     gen  = compile_program(FurutaHardware; Ts = 0.005)
#     ctrl = ProgramRuntime(gen; backend = :c, umax = 5.0)
#     run_inprocess!(ctrl; Tf = 10)

using ControlSystemsMTK: named_ss
using ControlSystemsBase: c2d, ss, lqr
using LinearAlgebra: Diagonal, I, pinv

export design_lqr, swingup_log

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

# Node outputs, in the order the runtime reports them. `row` first so the cheapest check --
# did the program write one row per tick -- is the first thing available. The MPC programs
# expose the same four signals of the same components and take them from here.
_swingup_outputs(nsys) = [nsys.logger.row, nsys.measurement.shoulder_angle,
                          nsys.measurement.elbow_angle, nsys.command.u_applied]

# The two parameters that stay settable at runtime are root parameters of `FurutaHardware`:
# `umax` is bound down into the stabilizer and the command clamp with `final`, `L` is the
# stabilizer's own (nothing binds it). A `ParametersStruct` field has to be unbound, which is
# what makes the root the right place for `umax` -- see `resolve_tunables`. The controller uses
# the `SwingupCatch` model's tuned defaults (energy-swingup gain, arm-centering, LQR gains and
# saturations), set for the `QubePendulum` plant with the identified parameters; the rest is
# changed with `overrides` to `compile_program`, using Dyad's `__`-separated paths (e.g.
# `control_system__runtime__swingup_catch__energyswingup__umax = 2.5`).
program_spec(::typeof(FurutaHardware)) = ProgramSpec(;
    name = :controller,
    tunables = OrderedDict{Any, Symbol}(
        (nsys -> nsys.control_system.runtime.swingup_catch.lqrstabilizer.L) => :L,
        (nsys -> nsys.umax) => :umax),
    outputs = _swingup_outputs,
    output_names = (:row, :shoulder, :elbow, :u),
    log = swingup_log)
