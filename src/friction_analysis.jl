# Implementation of the FurutaFrictionExperiment analysis: run the constant-velocity
# friction experiment on the physical QUBE.
#
# `FurutaFrictionExperiment` is a concrete `analysis` in `dyad/friction_experiment.dyad`;
# the Dyad compiler generates its spec/entry point, whose `run_analysis` forwards to
# `FurutaFrictionBaseSpec` (defined in friction_analysis_base.jl). This file provides
# `run_analysis(::FurutaFrictionBaseSpec)` — compiling `FurutaFriction` with the spec's
# `log_file` and running it against the hardware — plus the solution type and its artifacts.
#
# The estimation itself is deliberately not here: fitting is exploratory work that wants
# plots, data selection by eye and a choice of regressors, which is what
# examples/friction_identification.jl is for. This analysis produces the trace it reads.

export FurutaFrictionSolution

"""
    FurutaFrictionSolution

Result of the `FurutaFrictionExperiment` analysis: the `log_file` written, how many `rows`
the program wrote and over how many `ticks` (these should be equal — the program logs from
inside the tick), the `duration` in seconds, and the achieved loop period as
`(; median_dt, max_dt)`.

`ran` is `false` when the analysis was run with `run = false`, which compiles the program
and stops; the log is then whatever was there before, and the `:Trace` artifact is
unavailable.
"""
struct FurutaFrictionSolution{SP <: AbstractFurutaFrictionBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    log_file::String
    ran::Bool
    rows::Int
    ticks::Int
    duration::Float64
    timing::NamedTuple{(:median_dt, :max_dt), Tuple{Float64, Float64}}
end

function DyadInterface.run_analysis(spec::FurutaFrictionBaseSpec)
    backend = Symbol(spec.backend)
    backend in (:julia, :c) ||
        throw(ArgumentError("backend must be \"julia\" or \"c\", got $(repr(spec.backend))"))
    # One full up-and-down sweep unless asked otherwise. Anything shorter truncates the
    # reference schedule, which costs the fit its highest speeds or one whole direction.
    Tf = spec.Tf > 0 ? spec.Tf :
         friction_sweep_duration(; n_levels = spec.n_levels, t_step = spec.t_step)
    # `log_file` reaches the model, not only the driver: it is the `DataLogger`'s structural
    # `filename`, so the component that writes the file and the call that opens it cannot
    # disagree about which file that is.
    ctrl = FrictionController(; Ts = spec.Ts, backend, log_file = spec.log_file,
                                K = spec.K, Ti = spec.Ti,
                                umax = spec.umax, w_min = spec.w_min, w_max = spec.w_max,
                                n_levels = spec.n_levels, t_step = spec.t_step)
    spec.run || return FurutaFrictionSolution(spec, spec.log_file, false, 0, 0, 0.0,
                                              (; median_dt = NaN, max_dt = NaN))
    r = run_friction_experiment(ctrl; Tf, arm_deg = spec.arm_deg,
                                card_options = isempty(spec.card_options) ? nothing :
                                               spec.card_options)
    return FurutaFrictionSolution(spec, r.log_file, true, r.rows, r.ticks, Tf,
                                  (; median_dt = Float64(r.timing.median_dt),
                                     max_dt = Float64(r.timing.max_dt)))
end

function DyadInterface.AnalysisSolutionMetadata(sol::FurutaFrictionSolution)
    arts = ArtifactMetadata[]
    if sol.ran
        push!(arts, ArtifactMetadata(:Trace, ArtifactType.DataFrame,
            "Friction experiment trace",
            "What the program logged from inside each tick: elapsed time [s], the velocity \
             reference and the estimated shoulder velocity [rad/s], the shoulder and elbow \
             angles [rad], and the applied motor voltage [V]. Fit a friction model to this \
             with examples/friction_identification.jl."))
    end
    AnalysisSolutionMetadata(arts, Symbol[])
end

# `:Trace` returns the log parsed into a Tables.jl column table, with the column names the
# program wrote as the header.
function DyadInterface.artifacts(sol::FurutaFrictionSolution, name::Symbol)
    name === :Trace || throw(ArgumentError("Unknown artifact `$name`"))
    (sol.ran && isfile(sol.log_file)) ||
        throw(ArgumentError("No trace available (run the analysis with `run = true`)"))
    return read_friction_log(sol.log_file)
end

"""
    read_friction_log(path) -> NamedTuple of vectors

Parse a log written by the `DataLogger` inside `FurutaFriction`: a tab-separated header line
naming the columns, then one numeric row per tick. Returns a Tables.jl-compatible column
table keyed by those names, so it stays correct if `FRICTION_LOG_COLUMNS` changes.

A row cut short by the run being interrupted is dropped rather than erroring — the log is
flushed on close, so the last line may be partial.
"""
function read_friction_log(path)
    lines = readlines(path)
    isempty(lines) && throw(ArgumentError("$path is empty"))
    cols = Symbol.(split(strip(lines[1]), '\t'))
    rows = Vector{Vector{Float64}}()
    for l in Iterators.drop(lines, 1)
        isempty(strip(l)) && continue
        vals = tryparse.(Float64, split(strip(l), '\t'))
        (length(vals) == length(cols) && all(!isnothing, vals)) || continue
        push!(rows, Float64[vals...])
    end
    isempty(rows) && return NamedTuple{Tuple(cols)}(ntuple(_ -> Float64[], length(cols)))
    mat = reduce(vcat, permutedims.(rows))
    return NamedTuple{Tuple(cols)}(ntuple(j -> mat[:, j], length(cols)))
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaFrictionSolution)
    print(io, "FurutaFrictionExperiment solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    println(io, "log file: ", sol.log_file)
    if sol.ran
        println(io, "rows/ticks: ", sol.rows, " / ", sol.ticks,
                sol.rows == sol.ticks ? "" : "   (MISMATCH — the log lost rows)")
        println(io, "duration: ", sol.duration, " s")
        println(io, "loop period: median ", sol.timing.median_dt,
                " s, max ", sol.timing.max_dt, " s")
        println(io, "fit a model with examples/friction_identification.jl")
    else
        println(io, "not run (run = false); the program was compiled only")
    end
end
