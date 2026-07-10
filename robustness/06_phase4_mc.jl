# Phase 4: paired Monte Carlo comparison of the controller configurations.
# The same RNG seed is used in 02_montecarlo.jl, so all configs see the exact
# same 300 plant-parameter draws (paired comparison). tf = 20 s gives the
# adaptive configs a realistic time budget (adaptation needs extra swings);
# the baseline is re-run at the same tf so the comparison is fair. The
# earlier tf = 15 baseline result is preserved under a separate name.

f15 = joinpath(@__DIR__, "results", "mc_baseline.csv")
if isfile(f15) && !isfile(joinpath(@__DIR__, "results", "mc_baseline_tf15.csv"))
    cp(f15, joinpath(@__DIR__, "results", "mc_baseline_tf15.csv"))
end

tf_mc = 20.0
N_mc = 300

for config in ["baseline", "margin", "adaptive", "adaptive_ab"]
    global controller_config = config
    include("02_montecarlo.jl")
end

include("05_compare.jl")
