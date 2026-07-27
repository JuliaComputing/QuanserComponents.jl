# PR #3 re-analysis under the identified parameter set.
# Paired Monte Carlo: identical draws (seed 1) for every controller config.
tf_mc = 20.0
N_mc  = get(ENV, "N_MC", "300") |> x -> parse(Int, x)
for config in ["baseline", "margin", "adaptive", "adaptive_ab"]
    global controller_config = config
    include(joinpath(@__DIR__, "02_montecarlo.jl"))
end
include(joinpath(@__DIR__, "05_compare.jl"))
