# Shared harness for the swingup robustness campaigns.
#
# Conventions:
# - Plant parameters are perturbed while the controller keeps its nominal
#   parameter values, expressing plant-model mismatch.
# - Derived plant parameters (Jr, l, Jp) are baked numeric at model
#   instantiation, so they must be recomputed here whenever the leaf
#   parameters they derive from are perturbed.
# - The Lustre-compiled system holds a single shared executable, so all
#   campaigns run serially in-process (no threaded ensembles).

using QuanserComponents
using ModelingToolkit
using MultibodyComponents
using SynchToolkit
using OrdinaryDiffEqLowOrderRK
using DataFrames, CSV
using Printf

const SciMLBase = ModelingToolkit.SciMLBase

const Ts = 0.005

function build_system(; delayed = false, delay_n = 1)
    model = if delayed
        QuanserComponents.FurutaSwingupDelayed(; name = :model, delay_n)
    else
        QuanserComponents.FurutaSwingup(; name = :model)
    end
    ssys = multibody(model, additional_passes = [SynchToolkit.compile_lustre])
    model, ssys
end

"""
Controller settings pinning the ORIGINAL (pre-robustification) controller,
regardless of the current component defaults: unnormalized energy error,
no adaptation, original velocity estimator. This is the campaign "baseline"
and the reference for the behavior-neutrality regression.
"""
function controller_settings(ssys)
    Pair[
        ssys.qubependulum.shoulder_joint.render => false
        ssys.gain.k => 1.0
        ssys.swingup.lqrstabilizer.umax => 10
        ssys.swingup.energyswingup.umax => 3.0
        ssys.swingup.energyswingup.k => 100.0
        ssys.swingup.energyswingup.k_center => 1.0
        ssys.swingup.energyswingup.normalize => false
        ssys.swingup.energyswingup.eta => 0.0
        ssys.swingup.erefadaptation.gamma => 0.0
        ssys.swingup.estimatorswitch_shoulder.use_new => false
        ssys.swingup.estimatorswitch_elbow.use_new => false
    ]
end

"Initial conditions for a swingup experiment: pendulum hanging near down."
function swingup_ics(ssys)
    Pair[
        ssys.qubependulum.elbow_joint.phi => deg2rad(0.15)
        ssys.qubependulum.shoulder_joint.phi => 0.0
    ]
end

"Controller and initial-condition overrides matching test/runtests.jl."
nominal_overrides(ssys) = vcat(controller_settings(ssys), swingup_ics(ssys))

"""
Plant-parameter overrides expressing plant-model mismatch. The controller is
left at nominal. Derived parameters are recomputed from the perturbed leaves
since their baked values do not update automatically.

Coulomb friction and quantization arguments are only emitted when the model
supports them (added in Phase 1); pass `nothing` to omit.
"""
function perturbation_overrides(ssys;
        mp = 0.024, Lp = 0.129, mr = 0.095, r = 0.085,
        kt = 0.042, km = 0.042, Rm = 8.4,
        br = 5e-5, bp = 2.5e-6,
        tau_c_sh = nothing, tau_c_el = nothing, quantized = nothing)
    q = ssys.qubependulum
    ov = Pair[
        q.mp => mp
        q.Lp => Lp
        q.mr => mr
        q.r => r
        q.kt => kt
        q.km => km
        q.Rm => Rm
        q.br => br
        q.bp => bp
        # derived, baked at instantiation:
        q.Jp => mp * Lp^2 / 3
        q.l => Lp / 2
        q.Jr => mr * r^2 / 3
    ]
    tau_c_sh === nothing || push!(ov, q.shoulder_friction.tau_c => tau_c_sh)
    tau_c_el === nothing || push!(ov, q.elbow_friction.tau_c => tau_c_el)
    if quantized !== nothing
        push!(ov, ssys.elbow_sampler.quantized => quantized)
        push!(ov, ssys.shoulder_sampler.quantized => quantized)
    end
    ov
end

function simulate(ssys, overrides; tf = 10.0)
    prob = ODEProblem(ssys, overrides, (0.0, tf))
    solve(prob, BS3(), dt = Ts)
end

Base.@kwdef struct SimMetrics
    success::Bool
    catch_time::Float64    # last entry into the catch band that persists to the end
    first_entry::Float64   # first time the elbow angle enters the catch band
    n_entries::Int         # number of entries into the catch band (chattering metric)
    arrival_speed::Float64 # |elbow velocity| at first catch-band entry
    final_angle_err::Float64
    final_speed::Float64
    max_abs_elbow::Float64 # for the quantizer-range assertion (must stay < 4π)
end

"""
Classify a closed-loop solution. Success requires a successful solve without
NaNs and, over the final `window` seconds, the elbow angle within `angle_tol`
of upright with speed below `vel_tol`. The catch band uses the same threshold
as the NearTop component (evaluated on the true plant angle, a proxy for the
sampled signal the controller sees).
"""
function metrics(sol, ssys; window = 2.0, angle_tol = 0.15, vel_tol = 0.5, th = 0.4)
    q = ssys.qubependulum
    failed = SimMetrics(success = false, catch_time = NaN, first_entry = NaN,
        n_entries = 0, arrival_speed = NaN, final_angle_err = NaN,
        final_speed = NaN, max_abs_elbow = NaN)
    SciMLBase.successful_retcode(sol) || return failed

    t = range(0.0, sol.t[end], step = 2Ts)
    phi = sol(t, idxs = q.elbow_joint.phi).u
    w = sol(t, idxs = q.elbow_joint.w).u
    (any(isnan, phi) || any(isnan, w)) && return failed

    upright_err = @. abs(mod(phi, 2pi) - pi)
    inband = upright_err .< th

    n_entries = 0
    first_entry = NaN
    catch_time = NaN
    arrival_speed = NaN
    for i in 2:length(t)
        if inband[i] && !inband[i-1]
            n_entries += 1
            if isnan(first_entry)
                first_entry = t[i]
                arrival_speed = abs(w[i])
            end
            catch_time = t[i]
        end
    end
    if !isempty(inband) && inband[1] && n_entries == 0
        # started inside the band and never left/re-entered
        first_entry = catch_time = 0.0
        arrival_speed = abs(w[1])
        n_entries = 1
    end
    inband[end] || (catch_time = NaN)

    iwin = findall(>=(sol.t[end] - window), t)
    settled = all(upright_err[iwin] .< angle_tol) && all(abs.(w[iwin]) .< vel_tol)

    SimMetrics(
        success = settled && !isnan(catch_time),
        catch_time = catch_time,
        first_entry = first_entry,
        n_entries = n_entries,
        arrival_speed = arrival_speed,
        final_angle_err = upright_err[end],
        final_speed = abs(w[end]),
        max_abs_elbow = maximum(abs, phi),
    )
end

metrics_failed() = SimMetrics(success = false, catch_time = NaN, first_entry = NaN,
    n_entries = 0, arrival_speed = NaN, final_angle_err = NaN,
    final_speed = NaN, max_abs_elbow = NaN)

"True once the plant model contains the Coulomb-friction mismatch components."
function supports_friction(ssys)
    try
        ssys.qubependulum.shoulder_friction
        true
    catch
        false
    end
end

"Append `extra` to `base`, replacing any base entry with a matching key."
function with_overrides(base::Vector{<:Pair}, extra::Vector{<:Pair})
    ks = first.(extra)
    vcat(filter(p -> !any(k -> isequal(p.first, k), ks), base), extra)
end

"""
Named controller configurations compared in the campaigns. Each returns the
full controller override set; plant perturbations are appended separately.
"baseline" is the controller from test/runtests.jl. Robustified configs are
added in Phase 4.
"""
function controller_overrides(ssys, config::AbstractString)
    base = nominal_overrides(ssys)
    config == "baseline" && return base
    esw = ssys.swingup.energyswingup
    # Nominal upright energy assumed by the controller; with normalize on,
    # the gain acts on E/E_ref - 1, so rescale k by E_ref for equal authority.
    E_ref = 2 * 0.024 * 9.81 * 0.129 / 2
    if config == "margin"
        # A: normalized error + static reference margin
        return with_overrides(base, Pair[
            esw.normalize => true,
            esw.eta => 0.05,
            esw.k => 100.0 * E_ref,
        ])
    elseif config == "adaptive"
        # A + B: normalized error + online reference adaptation (no static margin)
        return with_overrides(base, Pair[
            esw.normalize => true,
            esw.k => 100.0 * E_ref,
            ssys.swingup.erefadaptation.gamma => 0.05,
        ])
    elseif config == "adaptive_ab"
        # A + B + C: additionally the alpha-beta velocity estimator, tuned
        # against quantization noise in phase3c_smoke.jl
        ab_alpha = 0.85
        ab_beta = ab_alpha^2 / (2 - ab_alpha)
        return with_overrides(base, Pair[
            esw.normalize => true,
            esw.k => 100.0 * E_ref,
            ssys.swingup.erefadaptation.gamma => 0.05,
            ssys.swingup.estimatorswitch_shoulder.use_new => true,
            ssys.swingup.estimatorswitch_elbow.use_new => true,
            ssys.swingup.alphabeta_shoulder.alpha => ab_alpha,
            ssys.swingup.alphabeta_shoulder.beta => ab_beta,
            ssys.swingup.alphabeta_elbow.alpha => ab_alpha,
            ssys.swingup.alphabeta_elbow.beta => ab_beta,
        ])
    end
    error("unknown controller config: $config")
end

function metrics_row(m::SimMetrics; extra...)
    merge(NamedTuple(extra), (;
        m.success, m.catch_time, m.first_entry, m.n_entries,
        m.arrival_speed, m.final_angle_err, m.final_speed, m.max_abs_elbow))
end

resultpath(name) = joinpath(@__DIR__, "results", name)
figurepath(name) = joinpath(@__DIR__, "figures", name)
