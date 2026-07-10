# Phase 3B verification:
# 1. Isolated ErefAdaptationTest model: eta ramps at swing apexes and clamps.
# 2. Full model regression: gamma = 0 default is behavior-neutral (00_baseline).
# 3. Feature-on: gamma > 0 rescues a mismatch case that fails at baseline.

include("common.jl")

# --- isolated component test ---
tmodel = QuanserComponents.ErefAdaptationTest(; name = :tmodel)
tsys = multibody(tmodel, additional_passes = [SynchToolkit.compile_lustre])
tprob = ODEProblem(tsys, Pair[], (0.0, 20.0))
tsol = solve(tprob, BS3(), dt = Ts)
eta = tsol[tsys.era.eta]
println("isolated: eta(0)=$(first(eta))  eta(end)=$(last(eta))  max=$(maximum(eta))")
@assert first(eta) == 0
@assert last(eta) > 0.3 "eta did not ramp as expected"
@assert maximum(eta) <= 0.5 + 1e-9 "eta exceeded eta_max"
println("isolated ErefAdaptation test passed")

# --- feature-on rescue test ---
# The mildest failing sample from the baseline MC campaign (sample 42):
# near-nominal masses and motor constants, but heavy viscous + Coulomb
# friction and quantization; the pendulum never reaches the catch band.
model, ssys = build_system()
hard = (; mp = 0.0244341173211006, Lp = 0.12963014151022853,
    mr = 0.08900340167288685, kt = 0.044360644011252365,
    km = 0.03913861399268383, Rm = 7.97187181423963,
    br = 0.0005855064113230941, bp = 4.786994165706217e-5,
    tau_c_sh = 0.0040382876985633346, tau_c_el = 0.0003770799004443717,
    quantized = true)

ov = vcat(nominal_overrides(ssys), perturbation_overrides(ssys; hard...))
m0 = metrics(simulate(ssys, ov; tf = 15.0), ssys)
println("hard case, baseline: success=$(m0.success) catch=$(m0.catch_time)")

for (gamma, tf) in ((0.05, 15.0), (0.05, 25.0), (0.1, 15.0))
    ov = vcat(with_overrides(nominal_overrides(ssys),
            Pair[ssys.swingup.erefadaptation.gamma => gamma]),
        perturbation_overrides(ssys; hard...))
    m1 = metrics(simulate(ssys, ov; tf), ssys)
    println("hard case, adaptive gamma=$gamma tf=$tf: success=$(m1.success) catch=$(m1.catch_time) entries=$(m1.n_entries)")
end

println("phase 3B smoke done")
