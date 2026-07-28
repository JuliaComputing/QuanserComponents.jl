# Base spec for the FurutaExportC analysis.
#
# `dyad/furuta_export_c.dyad` declares a `partial analysis FurutaExportCBase` (the
# parameter set) and a concrete `analysis FurutaExportC extends FurutaExportCBase()`. The
# Dyad compiler generates `FurutaExportCSpec <: AbstractFurutaExportCBaseSpec` plus a
# `run_analysis(::FurutaExportCSpec)` that forwards to `FurutaExportCBaseSpec` (see
# generated/FurutaExportC_definition.jl). This file defines that base spec + abstract
# supertype; it MUST be included before the generated module. The actual implementation
# (`run_analysis(::FurutaExportCBaseSpec)`, solution, artifacts) lives in
# `export_analysis.jl`, included after the model codegen helpers.

import DyadInterface
using ModelingToolkit: System, SymbolicT

abstract type AbstractFurutaExportCBaseSpec <: DyadInterface.AbstractAnalysisSpec end

@kwdef mutable struct FurutaExportCBaseSpec <: AbstractFurutaExportCBaseSpec
    name::Symbol = :FurutaExportCBase
    overrides::Dict{SymbolicT, SymbolicT} = Dict{SymbolicT, SymbolicT}()
    output_dir::String = "furuta_c"
    Ts::Float64 = 0.005
    Q1::Vector{Float64} = [1000.0, 10.0, 1.0, 1.0]
    Q2::Float64 = 100.0
    umax::Float64 = 10.0
    run::Bool = false
    Tf::Float64 = 10.0
    live_plot::Bool = false
    live_plot_cmd::String = "kst2"
    live_plot_config::String = "kst2config.kst"
    deploy_host::String = ""
    deploy_dir::String = "furuta_c"
    arm_deg::Float64 = 0.0
    model::Union{Nothing, System} = nothing
end
