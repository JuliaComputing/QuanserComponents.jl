# Feature-on sanity for Phase 3A: normalized energy error with a small
# static margin must still swing up on the nominal plant. With normalize on,
# the gain acts on E/E_ref - 1 instead of E - E_ref, so the equivalent gain
# is k*E_ref.

include("common.jl")

model, ssys = build_system()
@assert has_equation_swingup(ssys)

E_ref = 2 * 0.024 * 9.81 * 0.129 / 2

ov = with_overrides(nominal_overrides(ssys), Pair[
    ssys.swingup.energyswingup.normalize => true,
    ssys.swingup.energyswingup.eta => 0.05,
    ssys.swingup.energyswingup.k => 100.0 * E_ref,
])
m = metrics(simulate(ssys, ov; tf = 10.0), ssys)
println("normalize+eta: success=$(m.success) catch=$(m.catch_time) arrival=$(m.arrival_speed)")
@assert m.success "normalized controller failed to swing up nominally"
println("phase 3A smoke test passed")
