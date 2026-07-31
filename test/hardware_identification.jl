#=
Collect identification data on the physical QUBE: replay the input trajectory designed by
examples/input_design.jl and record what the device did, for fitting the model's parameters
with examples/pendulum_identification.jl.

Everything this script used to do by hand is now the `FurutaIdentificationExperiment`
analysis and the `FurutaIdentification` program it builds:

  * the open-loop replay -- `TrajectorySource` reads one designed sample per tick from
    inside the program, so the sequence is replayed identically whether the program runs
    in this process, as in-process C, or as exported C on the machine the QUBE is
    attached to;
  * the safety supervisor -- `SafetySupervisor` pulls the arm back past `warn_deg` and
    latches at `abort_deg`, commanding 0 for the rest of the run. The program does that
    itself now, so an abort no longer leaves the last commanded voltage on the motor
    while a driver unwinds;
  * the log -- `DataLogger` writes it from inside the tick, in the same 4-column layout
    examples/pendulum_identification.jl and examples/analyze_discrimination.jl read, plus
    the designed voltage, the supervisor's latch and the loop timing.

So there is nothing to keep in the loop here, and nothing left that needs
QuanserInterface: the device I/O is csrc/qube_hw.c, called from inside the program.

ENVIRONMENT: run in the package `test/` environment:
  julia --project=test test/hardware_identification.jl
=#

using QuanserComponents
using Printf

# The concrete analysis carries the deploy host, so this runs on whichever machine the QUBE
# is attached to and fetches the log back. Add `live_plot = true` (with a session file for
# these columns) to watch it as it happens.
sol = QuanserComponents.FurutaIdentificationExperiment(; run = true)

display(sol)

D = sol.data
if D !== nothing
    @printf("\n%s: %d samples, %.1f s, arm in [%.1f, %.1f] deg, pendulum in [%.1f, %.1f] deg, |u| <= %.2f V\n",
            sol.log_file, length(D.time), length(D.time) * sol.spec.Ts,
            rad2deg(minimum(D.shoulder_angle)), rad2deg(maximum(D.shoulder_angle)),
            rad2deg(minimum(D.elbow_angle)), rad2deg(maximum(D.elbow_angle)),
            maximum(abs, D.control_input))
    # How far the supervisor had to intervene, which the designed sequence should not need.
    n_sup = count(!≈(0; atol = 1e-9), D.control_input .- D.u_des)
    @printf("supervisor changed the command on %d of %d samples (%.1f%%)\n",
            n_sup, length(D.time), 100 * n_sup / length(D.time))
end
