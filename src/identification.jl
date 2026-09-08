# The open-loop identification replay as a synchronous program: what is specific to it.
#
# `FurutaIdentification` (dyad/identification.dyad) writes a designed voltage sequence to the
# motor and records what the device did, which is the input-output pair a parameter fit needs
# (examples/input_design.jl designs the sequence, examples/pendulum_identification.jl fits the
# model to the result). Everything is inside the program: `TrajectorySource` reads the sample,
# `SafetySupervisor` decides what is safe to write, and `DataLogger` writes the row.
#
# The generated node has the runtime signature
#     (row, k, u_des, u_applied, tripped) = step(tick, gains, auto)
# where `gains` carries the safety limits `umax`/`pullback` (runtime-settable, so a replay can
# be made gentler without recompiling) and `auto` the rest. `k` is the trajectory index, so a
# driver can see how far the replay got, and `tripped` says whether the supervisor latched.
#
# Compiling, the runtime and the timing loop are shared with the other programs and live in
# program.jl; getting any of them onto hardware is harness.jl's job. What is about this one is
# the `ProgramSpec` below:
#
#     gen  = compile_program(FurutaIdentification; Ts = 0.005, traj_file = "input_design.csv")
#     ctrl = ProgramRuntime(gen; umax = 3.0)
#     run_inprocess!(ctrl; Tf = 60)
#
# `Ts` must be the rate the trajectory was designed for: the program replays one sample per tick
# and does not resample. `traj_file` and `log_file` go to the model, not just to the driver --
# they are the `TrajectorySource`'s and `DataLogger`'s structural parameters -- so the components
# that read and write the files and the calls that open them cannot disagree about which files
# those are.

export identification_log, identification_traj

"The log `FurutaIdentification` writes, in `IDENTIFICATION_LOG_COLUMNS` order."
identification_log(file = IDENTIFICATION_LOG_FILE) =
    ProgramLog(file, IDENTIFICATION_LOG_COLUMNS)

"The input sequence `FurutaIdentification` replays; `examples/input_design.jl` writes it."
identification_traj(file = IDENTIFICATION_TRAJ_FILE, column = IDENTIFICATION_TRAJ_COLUMN) =
    ProgramTrajectory(file, column)

# The two safety values worth changing between runs without a recompile are root parameters of
# `FurutaIdentification`, bound down into the supervisor (and, for `umax`, the command clamp)
# with `final`, which is what a `ParametersStruct` field has to be -- see `resolve_tunables`.
# The outputs have `row` first so the cheapest check -- one row per tick -- is the first thing
# available; then how far the replay got, what it wanted, what it got, and whether the supervisor
# took over. The trajectory is built from the same keywords the model is, with the model's own
# defaults when they are not given.
program_spec(::typeof(FurutaIdentification)) = ProgramSpec(;
    name = :identification,
    tunables = OrderedDict{Any, Symbol}((nsys -> nsys.umax) => :umax,
                                        (nsys -> nsys.pullback) => :pullback),
    outputs = nsys -> [nsys.logger.row, nsys.trajectory.k, nsys.trajectory.u,
                       nsys.command.u_applied, nsys.supervisor.tripped],
    output_names = (:row, :k, :u_des, :u, :tripped),
    log = identification_log,
    traj = kw -> identification_traj(get(kw, :traj_file, IDENTIFICATION_TRAJ_FILE),
                                     get(kw, :traj_column, IDENTIFICATION_TRAJ_COLUMN)))
