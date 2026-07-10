# Recommendation 1 verification: the robustified configuration is now the
# component default.
# 1. The out-of-the-box model (no controller overrides at all) swings up.
# 2. Defaults + the campaign LQR authority reproduce the "adaptive_ab"
#    campaign configuration exactly.
# 3. The pinned "baseline" config still reproduces the original controller
#    (checked separately by 00_baseline.jl).

include("common.jl")

model, ssys = build_system()

ov = vcat(Pair[ssys.qubependulum.shoulder_joint.render => false], swingup_ics(ssys))
m_default = metrics(simulate(ssys, ov; tf = 10.0), ssys)
println("pure defaults: success=$(m_default.success) catch=$(m_default.catch_time)")
@assert m_default.success "out-of-the-box model failed to swing up"

m2 = metrics(simulate(ssys, controller_overrides(ssys, "adaptive_ab"); tf = 10.0), ssys)
println("defaults: catch=$(m_default.catch_time)  adaptive_ab config: catch=$(m2.catch_time)")
@assert m_default.catch_time == m2.catch_time "defaults do not reproduce the adaptive_ab campaign config"

println("rec 1 defaults smoke passed")
