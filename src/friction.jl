# The constant-velocity friction experiment as a synchronous program: what is specific to it.
#
# `FurutaFriction` (dyad/friction.dyad) is a purely discrete system that contains the whole
# experiment: `HardwareMeasurement` reads the encoders, a PI loop drives the arm around the
# `VelocityStaircase` reference, `HardwareCommand` writes the motor, and `DataLogger` writes a
# row per tick. `stkcompile` turns it into a synchronous node whose only argument is a clock
# tick -- so what is left out here is timing, and nothing else. In particular there is no
# logging code in the loop: the program writes the file.
#
# The generated node has the runtime signature
#     (row, w_ref, shoulder_angle, shoulder_velocity, u_applied) = step(tick, gains, auto)
# where `gains` carries the two velocity-loop parameters `K` and `Ti` (runtime-settable, so the
# loop can be retuned without recompiling) and `auto` the remaining model parameters. `row` is
# the logger's row count, returned so the driver can check that the program wrote one row per
# tick.
#
# Compiling, the runtime and the timing loop are shared with the other programs and live in
# program.jl; getting any program onto hardware is harness.jl's job. What is about this one is
# the `ProgramSpec` below:
#
#     gen  = compile_program(FurutaFriction; Ts = 0.005)
#     ctrl = ProgramRuntime(gen; K = 0.05, Ti = 0.5)
#     run_inprocess!(ctrl; Tf = friction_sweep_duration())
#
# Leave `card_options` at the driver's default for this experiment, which is what
# `run_inprocess!` and the analysis do unless told otherwise: the point of the experiment is to
# measure the friction *the controller faces*, so it has to run on the same command-to-torque
# path the controller does. Turning the driver's deadband compensation off here makes `kc`
# absorb the amplifier deadband, which then describes a plant that no run uses -- see the note in
# csrc/qube_hw.h for what that costs.

export friction_sweep_duration, friction_log

"The log `FurutaFriction` writes, in `FRICTION_LOG_COLUMNS` order. `file` defaults to `FRICTION_LOG_FILE`."
friction_log(file = FRICTION_LOG_FILE) = ProgramLog(file, FRICTION_LOG_COLUMNS)

# The integrator is forward-Euler: `I += (Ts/Ti) * e` each tick. `Ti` below a few sample times
# therefore moves the integrator by more than the error itself every tick, which does not
# track -- it oscillates or runs away. Worth saying out loud, because a too-small `Ti` looks
# like a tuning problem rather than a broken one. `Ni`, the PI loop's anti-windup gain, has a
# default expression in `Ti` and so is resolved once at compile time rather than tracking a later
# runtime change to `Ti` -- retuning `Ti` by a large factor is worth a recompile.
function _warn_integral_time(Ts, vals)
    Ti = vals[:Ti]
    Ti > 5 * Ts || @warn """
        Ti = $Ti is not much larger than Ts = $Ts, so the integrator gain Ts/Ti = \
        $(round(Ts / Ti, digits = 3)) per tick is very aggressive; the velocity loop will \
        not settle. Use Ti of at least a few tens of sample times.""" maxlog=1
    return
end

# `FurutaFriction`'s own `K`/`Ti`, not `velocity_pi`'s: the loop's are bound to these by
# `final K = K`, and a `ParametersStruct` field must be an *unbound* parameter. `velocity_pi.K`
# then has `K` as its default expression, which `AutoPars(gains)` resolves from the struct --
# the same mechanism `Ni` relies on. The outputs have `row` first so the row count is the
# cheapest thing to check; the rest is what the experiment is about, in the same order as the
# log's columns. Model parameters other than `K`/`Ti` are set with `overrides` to
# `compile_program`, e.g. `w_max = 20.0` for the model's own or `velocity_pi__wp = 1.0` for a
# sub-component's.
program_spec(::typeof(FurutaFriction)) = ProgramSpec(;
    name = :friction,
    tunables = OrderedDict{Any, Symbol}((nsys -> nsys.K) => :K, (nsys -> nsys.Ti) => :Ti),
    outputs = nsys -> [nsys.logger.row, nsys.reference.w_ref, nsys.measurement.shoulder_angle,
                       nsys.velocityestimator.vel, nsys.command.u_applied],
    output_names = (:row, :w_ref, :shoulder, :velocity, :u),
    log = friction_log,
    check = _warn_integral_time)

"""
    friction_sweep_duration(; n_levels=6, t_step=2.0) -> Float64

Seconds one full cycle of the `VelocityStaircase` reference takes: `n_levels` speeds in each
direction, `t_step` seconds each. This is the natural run length for the experiment --
shorter truncates the sweep, longer just repeats it.
"""
friction_sweep_duration(; n_levels = 6, t_step = 2.0) = 2 * float(n_levels) * float(t_step)
