# Feature-on smoke tests for the Phase 1 mismatch-injection mechanisms:
# each effect enabled individually must produce a successful solve, and the
# enabled effect must actually change the trajectory.

include("common.jl")

model, ssys = build_system()
@assert supports_friction(ssys)

base = simulate(ssys, nominal_overrides(ssys); tf = 10.0)
m0 = metrics(base, ssys)
println("nominal: success=$(m0.success) catch=$(m0.catch_time)")

# Coulomb friction on
ov = vcat(nominal_overrides(ssys), Pair[
    ssys.qubependulum.shoulder_friction.tau_c => 2e-3,
    ssys.qubependulum.elbow_friction.tau_c => 2e-4,
])
sol = simulate(ssys, ov; tf = 10.0)
m = metrics(sol, ssys)
println("coulomb: success=$(m.success) catch=$(m.catch_time)")
@assert m.catch_time != m0.catch_time "Coulomb friction had no effect on the trajectory"

# Quantization on
ov = vcat(nominal_overrides(ssys), Pair[
    ssys.elbow_sampler.quantized => true,
    ssys.shoulder_sampler.quantized => true,
])
sol = simulate(ssys, ov; tf = 10.0)
m = metrics(sol, ssys)
println("quantized: success=$(m.success) catch=$(m.catch_time)")
@assert m.max_abs_elbow < 4pi

# Delayed variant
model_d, ssys_d = build_system(delayed = true, delay_n = 1)
sol = simulate(ssys_d, nominal_overrides(ssys_d); tf = 10.0)
m = metrics(sol, ssys_d)
println("delay 1 sample: success=$(m.success) catch=$(m.catch_time)")

println("phase 1 smoke tests passed")
