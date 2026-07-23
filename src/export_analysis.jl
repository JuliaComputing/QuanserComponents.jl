# Dyad analysis `FurutaExportC`: export the swing-up controller as standalone C.
#
# The analysis designs the LQR feedback gain from the user-facing penalty weights
# `Q1` (state) and `Q2` (control) via [`design_lqr`](@ref), then generates the
# SynchToolkit C sources for the controller into `output_dir` via
# [`export_swingup_c`](@ref). The generated `StaticGains` struct carries the designed
# gain `L` and the saturation `umax` as its defaults.
#
# This is the runnable, package-shipped form (a `partial analysis` in
# `dyad/furuta_export_c.dyad`), mirroring `DyadFMUGeneration`: `run_analysis` writes
# files and reports them through a single table artifact.

import DyadInterface
using DyadInterface: AbstractAnalysisSpec, AbstractAnalysisSolution, ArtifactMetadata,
                     ArtifactType, AnalysisSolutionMetadata

export FurutaExportCSpec, FurutaExportC, FurutaExportCSolution

abstract type AbstractFurutaExportCSpec <: AbstractAnalysisSpec end

"""
    FurutaExportCSpec(; output_dir="furuta_c", Ts=0.005,
                        Q1=[1000.0, 10.0, 1.0, 1.0], Q2=100.0, umax=10.0)

Specification for the `FurutaExportC` analysis. Designs the LQR stabilizer gain from the
penalty weights `Q1` (state-cost diagonal, in the order `[shoulder_angle, elbow_angle,
shoulder_velocity, elbow_velocity]`) and `Q2` (scalar control cost), then exports the
swing-up controller as standalone C into `output_dir` at sample time `Ts`, with the
stabilizer saturation `umax`.
"""
@kwdef struct FurutaExportCSpec{M} <: AbstractFurutaExportCSpec
    name::Symbol = :FurutaExportC
    model::M = nothing
    output_dir::String = "furuta_c"
    Ts::Float64 = 0.005
    Q1::Vector{Float64} = [1000.0, 10.0, 1.0, 1.0]
    Q2::Float64 = 100.0
    umax::Float64 = 10.0
end

"""
    FurutaExportC(; kwargs...)

Convenience entry point equivalent to `run_analysis(FurutaExportCSpec(; kwargs...))`.
"""
FurutaExportC(; kwargs...) = DyadInterface.run_analysis(FurutaExportCSpec(; kwargs...))

"""
    FurutaExportCSolution

Result of [`FurutaExportC`](@ref): the `output_dir` written to, the list of generated
`files`, the `mangled` base name of the exported `<mangled>_step`/`_reset` C functions,
and the designed LQR gain `L`.
"""
struct FurutaExportCSolution{SP <: FurutaExportCSpec} <: AbstractAnalysisSolution
    spec::SP
    output_dir::String
    files::Vector{String}
    mangled::String
    L::Vector{Float64}
end

function DyadInterface.run_analysis(spec::FurutaExportCSpec)
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
