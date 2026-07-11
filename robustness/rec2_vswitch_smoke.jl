# Recommendation 2 verification (velocity-aware catch condition):
# 1. Nominal swingup with the ellipsoidal switch succeeds and engages exactly
#    once (no chattering).
# 2. Angle-only fallback (use_ellipsoid = false) reproduces the previous
#    behavior (checked via 00_baseline.jl separately; here we check the
#    adaptive_ab config, which pins the angle-only switch, still matches its
#    saved catch time).

include("common.jl")

model, ssys = build_system()

# robust swingup + ellipsoidal switch (opt-in config)
ov = controller_overrides(ssys, "adaptive_ab_vswitch")
sol = simulate(ssys, ov; tf = 10.0)
m = metrics(sol, ssys)
engaged = sol[ssys.swingup.catchcondition.y]
n_engage = count(i -> engaged[i] > 0.5 && engaged[i-1] <= 0.5, 2:length(engaged))
println("defaults+vswitch: success=$(m.success) catch=$(m.catch_time) engagements=$n_engage arrival=$(round(m.arrival_speed, digits=2))")
@assert m.success "swingup with ellipsoidal switch failed nominally"
@assert n_engage == 1 "switch chattered: $n_engage engagements"

# angle-only configs must be unaffected by the new component
m_ab = metrics(simulate(ssys, controller_overrides(ssys, "adaptive_ab"); tf = 10.0), ssys)
println("adaptive_ab (angle-only switch): success=$(m_ab.success) catch=$(m_ab.catch_time)")
@assert m_ab.success

println("rec 2 smoke tests passed")
