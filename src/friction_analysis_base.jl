# Base spec for the FurutaFrictionExperiment analysis.
#
# Same arrangement as export_analysis_base.jl: `dyad/friction_experiment.dyad` declares a
# `partial analysis FurutaFrictionBase` (the parameter set) and a concrete
# `analysis FurutaFrictionExperiment extends FurutaFrictionBase()`. The Dyad compiler
# generates `FurutaFrictionExperimentSpec <: AbstractFurutaFrictionBaseSpec` plus a
# `run_analysis` that forwards to `FurutaFrictionBaseSpec`, so this file MUST be included
# before the generated module. The implementation lives in `friction_analysis.jl`, included
# after the codegen helpers.
#
# `DyadInterface` and the `System`/`SymbolicT` names come from export_analysis_base.jl,
# included just before this. `FRICTION_LOG_FILE` is defined later still, in
# dyad/definitions.jl — fine, because `@kwdef` evaluates field defaults when a spec is
# constructed, not when the struct is defined.

abstract type AbstractFurutaFrictionBaseSpec <: DyadInterface.AbstractAnalysisSpec end

@kwdef mutable struct FurutaFrictionBaseSpec <: AbstractFurutaFrictionBaseSpec
    name::Symbol = :FurutaFrictionBase
    overrides::Dict{SymbolicT, SymbolicT} = Dict{SymbolicT, SymbolicT}()
    Ts::Float64 = 0.005
    log_file::String = FRICTION_LOG_FILE
    run::Bool = true
    Tf::Float64 = 0.0
    arm_deg::Float64 = 0.0
    card_options::String = "deadband_compensation=0.0"
    umax::Float64 = 10.0
    w_min::Float64 = 2.0
    w_max::Float64 = 30.0
    n_levels::Float64 = 6.0
    t_step::Float64 = 2.0
    spacing::Float64 = 2.0
    # Forwarded into the model by the concrete analysis' `model = FurutaFriction(final K = K,
    # final Ti = Ti, ...)`, so these arrive as `overrides` rather than being read from here.
    K::Float64 = 0.05
    Ti::Float64 = 0.5
    backend::String = "julia"
    settle::Float64 = 0.6
    acc_tol::Float64 = 3.0
    elbow_tol::Float64 = 1.5
    w_min_fit::Float64 = 0.5
    w_max_fit::Float64 = 40.0
    smooth_n::Int = 20
    model::Union{Nothing, System} = nothing
end
