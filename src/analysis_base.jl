# Spec bases for the analyses that build a synchronous program and run it on the QUBE.
#
# Both `FurutaExportC` and `FurutaFrictionExperiment` extend the partial analysis
# `QubeHardwareRunBase` (dyad/qube_hardware_run.dyad) for the parameters that describe
# building and running a program, and add their own on top. This file mirrors that
# arrangement on the Julia side: `@qube_run_spec` splices the shared field block into each
# base spec, so the shared parameters and their defaults are written once here and once in
# the Dyad partial rather than once per analysis.
#
# Included before the generated module, which needs the spec types and the abstract
# supertype to exist. The implementations live in export_analysis.jl and
# friction_analysis.jl, included after the codegen helpers.
#
# ## Why the root partial owns the entry point
#
# The Dyad compiler resolves an analysis' spec type to the *root* of its `extends` chain and
# carries it down unchanged (`rootAnalysis` in dyad-lang's
# pkgs/kernel/src/language/properties/analysis/merge.ts), so intermediate partials
# contribute parameters but no code. With a shared root, both analyses' generated entry
# points therefore construct `QubeHardwareRunBaseSpec` — the same name, whichever analysis
# was invoked. `QubeHardwareRunBaseSpec` is consequently a function rather than a struct: it
# picks the base spec that accepts the parameters it was handed, which keeps one typed
# `run_analysis` method per analysis.

import DyadInterface
using ModelingToolkit: System, SymbolicT

abstract type AbstractQubeHardwareRunBaseSpec <: DyadInterface.AbstractAnalysisSpec end

# The `QubeHardwareRunBase` parameter set, in the same order and with the same defaults as
# the Dyad partial. `overrides` is the model-parameter map the generated entry point builds
# out of the analysis' `model = Foo(final x = x)` line; `model` is the built system.
const _QUBE_RUN_FIELDS = [
    :(overrides::Dict{SymbolicT, SymbolicT} = Dict{SymbolicT, SymbolicT}()),
    :(Ts::Float64 = 0.005),
    :(run::Bool = false),
    :(Tf::Float64 = 0.0),
    :(umax::Float64 = 10.0),
    :(arm_deg::Float64 = 0.0),
    :(card_options::String = ""),
    :(backend::String = "julia"),
    :(export_c::Bool = false),
    :(output_dir::String = "furuta_c"),
    :(log_file::String = ""),
    :(deploy_host::String = ""),
    :(deploy_dir::String = "furuta_c"),
    :(live_plot::Bool = false),
    :(live_plot_cmd::String = "kst2"),
    :(live_plot_config::String = "kst2config.kst"),
    :(model::Union{Nothing, System} = nothing),
]

_spec_field_name(ex::Expr) = ex.head === :(=) ? _spec_field_name(ex.args[1]) : ex.args[1]

"""
    @qube_run_spec SpecName :base_name begin <extra fields> end

Define a mutable keyword-constructible analysis spec with the `QubeHardwareRunBase` fields
plus the ones given. A field that repeats a shared name replaces it, which is how an
analysis whose Dyad partial writes `extends QubeHardwareRunBase(export_c = true)` states
that default here as well.
"""
macro qube_run_spec(specname, base_name, extra)
    extras = filter(x -> !(x isa LineNumberNode), extra.args)
    overridden = Set(_spec_field_name.(extras))
    shared = filter(f -> !(_spec_field_name(f) in overridden), _QUBE_RUN_FIELDS)
    fields = [:(name::Symbol = $base_name); shared; extras]
    return esc(quote
        Base.@kwdef mutable struct $specname <: AbstractQubeHardwareRunBaseSpec
            $(fields...)
        end
    end)
end

@qube_run_spec FurutaExportCBaseSpec :FurutaExportCBase begin
    export_c::Bool = true
    Tf::Float64 = 10.0
    Q1::Vector{Float64} = [1000.0, 10.0, 1.0, 1.0]
    Q2::Float64 = 100.0
end

@qube_run_spec FurutaFrictionBaseSpec :FurutaFrictionBase begin
    run::Bool = true
    card_options::String = "deadband_compensation=0.0"
    output_dir::String = "friction_c"
    # Its own directory on the deploy host, so a deployed friction experiment does not
    # overwrite the swing-up controller's sources there (or get overwritten by them).
    deploy_dir::String = "friction_c"
    w_min::Float64 = 2.0
    w_max::Float64 = 30.0
    n_levels::Float64 = 6.0
    t_step::Float64 = 2.0
    spacing::Float64 = 2.0
    # Forwarded into the model by the analysis' `model = FurutaFriction(final K = K, ...)`,
    # so these arrive as `overrides` rather than being read out of here.
    K::Float64 = 0.05
    Ti::Float64 = 0.5
    settle::Float64 = 0.6
    acc_tol::Float64 = 3.0
    elbow_tol::Float64 = 1.5
    w_min_fit::Float64 = 0.5
    w_max_fit::Float64 = 40.0
    smooth_n::Int = 20
end

# Base specs an analysis' generated entry point may be asking for, most specific first.
const QUBE_RUN_SPECS = (FurutaExportCBaseSpec, FurutaFrictionBaseSpec)

"""
    QubeHardwareRunBaseSpec(; parameters...)

Build the base spec of whichever analysis these parameters belong to.

Not a struct, despite the name: every analysis extending `QubeHardwareRunBase` has this as
its generated entry point (see the file header), so this is where one entry point fans back
out into one spec type — and one `run_analysis` implementation — per analysis. The
parameters an analysis owns beyond the shared set are what identifies it, so the spec chosen
is the one whose fields cover everything passed. Ambiguity or a parameter no spec knows is an
error rather than a guess, since silently running the other analysis would be worse.
"""
function QubeHardwareRunBaseSpec(; name = nothing, kwargs...)
    given = keys(kwargs)
    match = filter(T -> issubset(given, fieldnames(T)), QUBE_RUN_SPECS)
    if length(match) != 1
        unknown = setdiff(given, union(fieldnames.(QUBE_RUN_SPECS)...))
        isempty(unknown) ||
            throw(ArgumentError("no analysis in QUBE_RUN_SPECS has the parameter(s) \
                                 $(join(unknown, ", ")); add its spec there"))
        throw(ArgumentError("the parameters $(join(given, ", ")) match \
                             $(length(match)) analyses ($(join(match, ", "))); they must \
                             identify exactly one"))
    end
    # `name` is deliberately dropped: the compiler passes the root partial's name, which is
    # the same for every analysis, so each spec's own default is the more informative one.
    return only(match)(; kwargs...)
end
