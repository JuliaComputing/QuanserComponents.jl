# Phase 3C verification:
# 1. Behavior-neutrality with use_new = false (00_baseline regression is run
#    separately; here we check the alpha-beta estimator is simulated alongside).
# 2. Velocity-estimate accuracy with quantization on: the alpha-beta estimate
#    must have lower RMS error against the true elbow velocity than the
#    derivative + low-pass estimator.
# 3. Feature-on: swingup succeeds nominally with use_new = true.

include("common.jl")
using Statistics

model, ssys = build_system()

function estimator_rms(; ab_alpha = 0.5)
    ab_beta = ab_alpha^2 / (2 - ab_alpha)
    ov = vcat(nominal_overrides(ssys), Pair[
        ssys.elbow_sampler.quantized => true,
        ssys.shoulder_sampler.quantized => true,
        ssys.swingup.alphabeta_elbow.alpha => ab_alpha,
        ssys.swingup.alphabeta_elbow.beta => ab_beta,
        ssys.swingup.alphabeta_shoulder.alpha => ab_alpha,
        ssys.swingup.alphabeta_shoulder.beta => ab_beta,
    ])
    sol = simulate(ssys, ov; tf = 10.0)
    t = range(0.5, 10.0, step = 2Ts)  # skip initial estimator transients
    w_true = sol(t, idxs = ssys.qubependulum.elbow_joint.w).u
    v_old = sol(t, idxs = ssys.swingup.velocityestimator_elbow.vel).u
    v_new = sol(t, idxs = ssys.swingup.alphabeta_elbow.vel).u
    sqrt(mean(abs2, v_old .- w_true)), sqrt(mean(abs2, v_new .- w_true))
end

results = map((0.5, 0.7, 0.85, 0.95)) do a
    rms_old, rms_new = estimator_rms(ab_alpha = a)
    println("alpha=$a: old=$(round(rms_old, digits=4)) new=$(round(rms_new, digits=4))")
    (a, rms_old, rms_new)
end
best = argmin(r -> r[3], results)
println("best alpha-beta tuning: alpha=$(best[1]) rms=$(round(best[3], digits=4)) vs old $(round(best[2], digits=4))")
@assert best[3] < best[2] "alpha-beta estimator is not more accurate than the old one at any tested tuning"

ab_alpha = 0.85
ab_beta = ab_alpha^2 / (2 - ab_alpha)
ov = vcat(with_overrides(nominal_overrides(ssys), Pair[
        ssys.swingup.estimatorswitch_shoulder.use_new => true,
        ssys.swingup.estimatorswitch_elbow.use_new => true,
        ssys.swingup.alphabeta_elbow.alpha => ab_alpha,
        ssys.swingup.alphabeta_elbow.beta => ab_beta,
        ssys.swingup.alphabeta_shoulder.alpha => ab_alpha,
        ssys.swingup.alphabeta_shoulder.beta => ab_beta,
    ]))
m = metrics(simulate(ssys, ov; tf = 10.0), ssys)
println("use_new swingup: success=$(m.success) catch=$(m.catch_time)")
@assert m.success "swingup with alpha-beta estimator failed nominally"

println("phase 3C smoke tests passed")
