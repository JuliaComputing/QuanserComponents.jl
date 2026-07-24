# Implementation of the FurutaExportC analysis: export the swing-up controller as C.
#
# `FurutaExportC` is a concrete `analysis` in `dyad/furuta_export_c.dyad`; the Dyad
# compiler generates its spec/entry point (generated/FurutaExportC_definition.jl), whose
# `run_analysis` forwards to `FurutaExportCBaseSpec` (defined in export_analysis_base.jl).
# This file provides `run_analysis(::FurutaExportCBaseSpec)` — designing the LQR feedback
# gain from the penalty weights `Q1`/`Q2` via `design_lqr`, then generating the SynchToolkit
# C sources into `output_dir` via `export_swingup_c` — plus the solution type and its
# artifacts. Results are reported through a single table artifact.

using DyadInterface: AbstractAnalysisSolution, ArtifactMetadata,
                     ArtifactType, AnalysisSolutionMetadata

export FurutaExportCSolution

"""
    FurutaExportCSolution

Result of the `FurutaExportC` analysis: the `output_dir` written to, the list of generated
`files`, the `mangled` base name of the exported `<mangled>_step`/`_reset` C functions, and
the designed LQR gain `L`. When the analysis was run with `run = true`, `ran` is `true` and
`log` is the path to the hardware run's CSV log (otherwise `nothing`).
"""
struct FurutaExportCSolution{SP <: AbstractFurutaExportCBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    output_dir::String
    files::Vector{String}
    mangled::String
    L::Vector{Float64}
    ran::Bool
    log::Union{Nothing, String}
end

function DyadInterface.run_analysis(spec::FurutaExportCBaseSpec)
    mkpath(spec.output_dir)
    L = design_lqr(; Ts = spec.Ts, Q1 = spec.Q1, Q2 = spec.Q2)
    res = export_swingup_c(spec.output_dir; Ts = spec.Ts, L = L, umax = spec.umax, Tf = spec.Tf)
    # `run = true` compiles the emitted C control loop and executes it on the physical
    # pendulum for `Tf` seconds, capturing the trace as the `:RunLog` artifact.
    log = nothing
    if spec.run
        exe = compile_hardware_harness(spec.output_dir)
        log = run_hardware_harness(exe)
    end
    files = sort!(filter(f -> isfile(joinpath(spec.output_dir, f)),
                         readdir(spec.output_dir)))
    return FurutaExportCSolution(spec, spec.output_dir, files, res.mangled, L, spec.run, log)
end

function DyadInterface.AnalysisSolutionMetadata(sol::FurutaExportCSolution)
    arts = [ArtifactMetadata(:GeneratedFiles, ArtifactType.DataFrame,
        "Generated C files",
        "The SynchToolkit-generated C sources for the swing-up controller, with the \
         exported step/reset symbol names.")]
    if sol.ran
        push!(arts, ArtifactMetadata(:RunLog, ArtifactType.DataFrame,
            "Hardware run log",
            "Time series logged while running the generated controller on the hardware: \
             time [s], shoulder/elbow angles [rad], the commanded control voltage [V], and \
             the timing diagnostics dt (achieved period [s]) and exec (loop-body time [s])."))
    end
    AnalysisSolutionMetadata(arts, Symbol[])
end

# `:GeneratedFiles` returns a column table (Tables.jl-compatible NamedTuple of vectors)
# listing each generated file, its size in bytes, and the exported C symbol on the
# source/header rows. `:RunLog` (only when the analysis was run) returns the hardware
# trace parsed from `run_hardware.csv`.
function DyadInterface.artifacts(sol::FurutaExportCSolution, name::Symbol)
    if name === :GeneratedFiles
        files = sol.files
        bytes = [filesize(joinpath(sol.output_dir, f)) for f in files]
        symbol = map(files) do f
            f == "top.c" ? "$(sol.mangled)_step" :
            f == "top.h" ? "$(sol.mangled)_reset" : ""
        end
        return (; file = files, bytes = bytes, symbol = symbol)
    elseif name === :RunLog
        (sol.ran && sol.log !== nothing && isfile(sol.log)) ||
            throw(ArgumentError("No run log available (run the analysis with `run = true`)"))
        return _read_run_log(sol.log)
    else
        throw(ArgumentError("Unknown artifact `$name`"))
    end
end

# Parse the harness-written log (tab-separated header + numeric rows) into a Tables.jl
# column table. Columns: time, shoulder_angle, elbow_angle, control_input, the timing
# diagnostics dt (achieved period) and exec (control-loop body duration) in seconds, and
# the raw encoder counts (for diagnosing counter glitches).
function _read_run_log(path)
    cols = (:time, :shoulder_angle, :elbow_angle, :control_input, :dt, :exec,
            :count_shoulder, :count_elbow)
    empty = NamedTuple{cols}(ntuple(_ -> Float64[], length(cols)))
    lines = readlines(path)
    length(lines) > 1 || return empty
    rows = [parse.(Float64, split(l)) for l in lines[2:end] if !isempty(strip(l))]
    isempty(rows) && return empty
    mat = reduce(vcat, permutedims.(rows))
    return NamedTuple{cols}(ntuple(j -> mat[:, j], length(cols)))
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaExportCSolution)
    print(io, "FurutaExportC solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    println(io, "output_dir: ", sol.output_dir)
    println(io, "files: ", join(sol.files, ", "))
    println(io, "step/reset: ", sol.mangled, "_step / ", sol.mangled, "_reset")
    println(io, "designed L: ", sol.L)
    if sol.ran
        println(io, "hardware run log: ", sol.log === nothing ? "(none)" : sol.log)
    end
end
