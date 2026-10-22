# The generated analyses share a root entry point. Their analysis-specific parameters
# are carried by `__system`; the hand-written specs provide the implementations.

import DyadInterface
import ModelingToolkit
using ModelingToolkit: System, SymbolicT

abstract type AbstractQubeHardwareRunBaseSpec <: DyadInterface.AbstractAnalysisSpec end

# The `QubeHardwareRunBase` parameter set, in the same order and with the same defaults as
# the Dyad partial. `overrides` is the model-parameter map the generated entry point builds
# out of the analysis' `model = Foo(final x = x)` line; `model` is the built system.
# `__system` carries the parameters declared by derived analyses.
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
    :(__system::Union{Nothing, System} = nothing),
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

@qube_run_spec FurutaSwingupBaseSpec :FurutaSwingupBase begin
    export_c::Bool = true
    Tf::Float64 = 10.0
    Q1::Vector{Float64} = [1000.0, 10.0, 1.0, 1.0]
    Q2::Float64 = 100.0
end

@qube_run_spec FurutaFrictionBaseSpec :FurutaFrictionBase begin
    run::Bool = true
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

@qube_run_spec FurutaIdentificationBaseSpec :FurutaIdentificationBase begin
    export_c::Bool = true
    umax::Float64 = 3.0
    output_dir::String = "identification_c"
    deploy_dir::String = "identification_c"
    traj_file::String = "input_design.csv"
    traj_column::Int = 1
    warn_deg::Float64 = 95.0
    abort_deg::Float64 = 120.0
    pullback::Float64 = 2.0
end

# Base specs an analysis' generated entry point may be asking for, most specific first.
const QUBE_RUN_SPECS = (FurutaSwingupBaseSpec, FurutaFrictionBaseSpec,
                        FurutaIdentificationBaseSpec)

# Derived parameter bindings must be evaluated against the analysis initial conditions.
function _system_values(sys::System)
    ics = ModelingToolkit.initial_conditions(sys)
    binds = ModelingToolkit.bindings(sys)
    env = merge(Dict(ics), Dict(binds))
    values = Dict{Symbol, Any}()
    for p in ModelingToolkit.parameters(sys)
        v = get(env, p, nothing)
        v === nothing && continue
        v = ModelingToolkit.Symbolics.value(
            ModelingToolkit.Symbolics.fixpoint_sub(v, env; fold = Val(true)))
        values[Symbol(ModelingToolkit.getname(p))] = v
    end
    return values
end

_coerce_field(::Type{T}, v) where {T <: AbstractVector} = T(collect(v))
_coerce_field(::Type{T}, v) where {T} = convert(T, v)

"""
    QubeHardwareRunBaseSpec(; parameters...)

Construct the unique analysis specification accepting the supplied parameters.

Analysis-specific parameters are read from `__system`, including derived bindings.
Explicit keyword arguments take precedence over values from that system. Ambiguous
parameter sets and unknown keywords raise an `ArgumentError`.
"""
function QubeHardwareRunBaseSpec(; name = nothing, __system = nothing, kwargs...)
    known = union(fieldnames.(QUBE_RUN_SPECS)...)
    sysvalues = __system isa System ? _system_values(__system) : Dict{Symbol, Any}()
    filter!(kv -> first(kv) in known, sysvalues)
    given = union(keys(kwargs), keys(sysvalues))
    match = filter(T -> issubset(given, fieldnames(T)), QUBE_RUN_SPECS)
    if length(match) != 1
        unknown = setdiff(given, known)
        isempty(unknown) ||
            throw(ArgumentError("no analysis in QUBE_RUN_SPECS has the parameter(s) \
                                 $(join(unknown, ", ")); add its spec there"))
        throw(ArgumentError("the parameters $(join(given, ", ")) match \
                             $(length(match)) analyses ($(join(match, ", "))); they must \
                             identify exactly one"))
    end
    T = only(match)
    # Keywords win over the system, whose values are those the analysis was constructed with.
    fields = Dict{Symbol, Any}(kwargs)
    for (f, v) in sysvalues
        haskey(fields, f) || (fields[f] = _coerce_field(fieldtype(T, f), v))
    end
    # `name` is deliberately dropped: each spec's own default is the more informative one.
    __system isa System && (fields[:__system] = __system)
    return T(; fields...)
end
