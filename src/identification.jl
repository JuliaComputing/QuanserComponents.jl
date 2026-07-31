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
# Compiling, the runtime and the timing loop are shared with the other two programs and live in
# program.jl; getting any of them onto hardware is harness.jl's job.

export generate_identification_controller, IdentificationController, identification_log,
       identification_traj

"The log `FurutaIdentification` writes, in `IDENTIFICATION_LOG_COLUMNS` order."
identification_log(file = IDENTIFICATION_LOG_FILE) =
    ProgramLog(file, IDENTIFICATION_LOG_COLUMNS)

"The input sequence `FurutaIdentification` replays; `examples/input_design.jl` writes it."
identification_traj(file = IDENTIFICATION_TRAJ_FILE, column = IDENTIFICATION_TRAJ_COLUMN) =
    ProgramTrajectory(file, column)

# The two safety values worth changing between runs without a recompile. Both are root
# parameters of `FurutaIdentification`, bound down into the supervisor (and, for `umax`, the
# command clamp) with `final`, which is what a `ParametersStruct` field has to be -- see
# `resolve_tunables`.
const IDENTIFICATION_TUNABLES = OrderedDict{Any, Symbol}(
    (nsys -> nsys.umax) => :umax,
    (nsys -> nsys.pullback) => :pullback,
)

# `row` first so the cheapest check -- one row per tick -- is the first thing available. Then
# how far the replay got, what it wanted, what it got, and whether the supervisor took over.
_identification_outputs(nsys) = [nsys.logger.row, nsys.trajectory.k, nsys.trajectory.u,
                                nsys.command.u_applied, nsys.supervisor.tripped]
const IDENTIFICATION_OUTPUT_NAMES = (:row, :k, :u_des, :u, :tripped)

"""
    generate_identification_controller(; Ts=0.005, traj_file=IDENTIFICATION_TRAJ_FILE,
                                        traj_column=2, log_file=IDENTIFICATION_LOG_FILE,
                                        overrides...)

Compile the open-loop replay to a SynchJulia node: build `FurutaIdentification` on a
`PeriodicClock` at sample time `Ts` and `stkcompile` it.

Returns what [`compile_program`](@ref) returns. `Ts` must be the rate the trajectory was
designed for — the program replays one sample per tick and does not resample.

`traj_file` and `log_file` go to the model, not just to the driver: they are the
`TrajectorySource`'s and `DataLogger`'s structural parameters, so the components that read and
write the files and the calls that open them cannot disagree about which files those are.
"""
function generate_identification_controller(; Ts = 0.005,
                                             traj_file = IDENTIFICATION_TRAJ_FILE,
                                             traj_column = IDENTIFICATION_TRAJ_COLUMN,
                                             log_file = IDENTIFICATION_LOG_FILE,
                                             param_overrides = nothing, overrides...)
    return compile_program(FurutaIdentification; name = :identification, Ts,
                           tunables = IDENTIFICATION_TUNABLES,
                           outputs = _identification_outputs,
                           log = identification_log(log_file),
                           traj = identification_traj(traj_file, traj_column),
                           traj_file, traj_column, param_overrides, overrides...)
end

"""
    IdentificationController(; Ts=0.005, backend=:julia, traj_file, log_file, umax=nothing,
                               pullback=nothing, overrides...)

A ready-to-call runtime wrapper around the generated replay. Compiles the program, builds a
`SynchExecutable` on `backend` (`:julia` or `:c`), and populates the parameter structs.

Advance one step with `out = controller()`, which reads the encoders, takes the next trajectory
sample, passes it through the safety supervisor, writes the motor and appends a row to the log,
returning `(; row, k, u_des, u, tripped)`. Open the device with [`open_hardware!`](@ref), the
log with [`open_log!`](@ref) and the trajectory with [`open_traj!`](@ref) first — or call
[`run_program!`](@ref), which does all three.

`umax` and `pullback` override the safety limits at runtime; both default to the model's.
"""
function IdentificationController(; Ts = 0.005, backend::Symbol = :julia,
                                   traj_file = IDENTIFICATION_TRAJ_FILE,
                                   traj_column = IDENTIFICATION_TRAJ_COLUMN,
                                   log_file = IDENTIFICATION_LOG_FILE,
                                   umax = nothing, pullback = nothing, kwargs...)
    gen = generate_identification_controller(; Ts, traj_file, traj_column, log_file, kwargs...)
    return make_runtime(gen, IDENTIFICATION_OUTPUT_NAMES; backend, gains = (; umax, pullback))
end
