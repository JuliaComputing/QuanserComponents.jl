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
the designed LQR gain `L`.
"""
struct FurutaExportCSolution{SP <: AbstractFurutaExportCBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    output_dir::String
    files::Vector{String}
    mangled::String
    L::Vector{Float64}
end

function DyadInterface.run_analysis(spec::FurutaExportCBaseSpec)
    mkpath(spec.output_dir)
    L = design_lqr(; Ts = spec.Ts, Q1 = spec.Q1, Q2 = spec.Q2)
    res = export_swingup_c(spec.output_dir; Ts = spec.Ts, L = L, umax = spec.umax)
    files = sort!(filter(f -> isfile(joinpath(spec.output_dir, f)),
                         readdir(spec.output_dir)))
    return FurutaExportCSolution(spec, spec.output_dir, files, res.mangled, L)
end

function DyadInterface.AnalysisSolutionMetadata(sol::FurutaExportCSolution)
    arts = [ArtifactMetadata(:GeneratedFiles, ArtifactType.DataFrame,
        "Generated C files",
        "The SynchToolkit-generated C sources for the swing-up controller, with the \
         exported step/reset symbol names.")]
    AnalysisSolutionMetadata(arts, Symbol[])
end

# Returns a column table (Tables.jl-compatible NamedTuple of vectors) listing each
# generated file, its size in bytes, and the exported C symbol on the source/header rows.
function DyadInterface.artifacts(sol::FurutaExportCSolution, name::Symbol)
    name === :GeneratedFiles || throw(ArgumentError("Unknown artifact `$name`"))
    files = sol.files
    bytes = [filesize(joinpath(sol.output_dir, f)) for f in files]
    symbol = map(files) do f
        f == "top.c" ? "$(sol.mangled)_step" :
        f == "top.h" ? "$(sol.mangled)_reset" : ""
    end
    return (; file = files, bytes = bytes, symbol = symbol)
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaExportCSolution)
    print(io, "FurutaExportC solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    println(io, "output_dir: ", sol.output_dir)
    println(io, "files: ", join(sol.files, ", "))
    println(io, "step/reset: ", sol.mangled, "_step / ", sol.mangled, "_reset")
    println(io, "designed L: ", sol.L)
end
