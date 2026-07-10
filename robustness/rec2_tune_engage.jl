# Calibrate the CatchCondition engage threshold on the MC draws that the
# strict (c_engage = 1) ellipsoid broke relative to the angle-only switch,
# plus the draws it rescued (to check they are not lost again).

include("common.jl")
using Random, Statistics

model, ssys = build_system()

ab = sort(CSV.read(resultpath("mc_adaptive_ab.csv"), DataFrame), :sample)
v1 = sort(CSV.read(resultpath("mc_adaptive_ab_vswitch.csv"), DataFrame), :sample)
broken = ab.sample[ab.success .& .!v1.success]
rescued = ab.sample[.!ab.success .& v1.success]
samples = vcat(broken, rescued)
println("tuning on $(length(broken)) broken + $(length(rescued)) rescued draws")

params = [:mp, :Lp, :mr, :kt, :km, :Rm, :br, :bp, :tau_c_sh, :tau_c_el, :quantized]

function run_sample(row, c_engage, c_release)
    kwargs = NamedTuple(p => row[p] for p in params)
    ov = vcat(
        with_overrides(controller_overrides(ssys, "adaptive_ab_vswitch"),
            Pair[ssys.swingup.catchcondition.c_engage => c_engage,
                 ssys.swingup.catchcondition.c_release => c_release]),
        perturbation_overrides(ssys; kwargs...))
    metrics(simulate(ssys, ov; tf = 20.0), ssys).success
end

for (c_engage, c_release) in ((2.0, 4.0), (2.0, 8.0), (3.0, 6.0), (1.5, 6.0))
    nb = count(run_sample(ab[ab.sample .== s, :][1, :], c_engage, c_release) for s in broken)
    nr = count(run_sample(ab[ab.sample .== s, :][1, :], c_engage, c_release) for s in rescued)
    println("c_engage=$c_engage c_release=$c_release: recovers $nb/$(length(broken)) broken, keeps $nr/$(length(rescued)) rescued")
end
