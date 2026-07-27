# Joint Monte Carlo campaign: all plant parameters perturbed simultaneously,
# controller at nominal. Reports overall success rate and per-parameter
# conditional failure rates.

include("common.jl")
using Random, Statistics

@isdefined(controller_config) || (controller_config = "baseline")
@isdefined(N_mc) || (N_mc = 300)
@isdefined(tf_mc) || (tf_mc = 15.0)

model, ssys = build_system()
ctrl = controller_overrides(ssys, controller_config)
has_friction = supports_friction(ssys)

rng = MersenneTwister(1)

runif(rng, lo, hi) = lo + (hi - lo) * rand(rng)
rlogunif(rng, lo, hi) = exp10(runif(rng, log10(lo), log10(hi)))

rows = NamedTuple[]
for i in 1:N_mc
    draw = (;
        mp = IDP.mp * runif(rng, 0.8, 1.2),
        Lp = IDP.Lp * runif(rng, 0.9, 1.1),
        mr = IDP.mr * runif(rng, 0.8, 1.2),
        kt = IDP.kt * runif(rng, 0.85, 1.15),
        km = IDP.km * runif(rng, 0.85, 1.15),
        Rm = IDP.Rm * runif(rng, 0.8, 1.2),
        # independent in the identified model (previously implied by mr/r/mp/Lp)
        Jr = IDP.Jr * runif(rng, 0.8, 1.2),
        Jp = IDP.Jp * runif(rng, 0.8, 1.2),
        r_cm_r = IDP.r_cm_r * runif(rng, 0.9, 1.1),
        br = IDP.br * rlogunif(rng, 0.1, 20),
        bp = BP_CENTER * rlogunif(rng, 0.1, 20),
        tau_c_sh = has_friction ? runif(rng, 0.0, 5e-3) : nothing,
        tau_c_el = has_friction ? runif(rng, 0.0, 5e-4) : nothing,
        quantized = has_friction ? rand(rng, Bool) : nothing,
    )
    m = try
        ov = vcat(ctrl, perturbation_overrides(ssys; draw...))
        metrics(simulate(ssys, ov; tf = tf_mc), ssys)
    catch err
        @warn "simulation crashed" i err
        metrics_failed()
    end
    clean = map(v -> v === nothing ? missing : v, draw)
    push!(rows, metrics_row(m; sample = i, controller = controller_config, clean...))
    @printf("%4d/%d success %s catch %6.2f\n", i, N_mc, m.success, m.catch_time)
end

df = DataFrame(rows)
CSV.write(resultpath("mc_$(controller_config).csv"), df)

rate = mean(df.success)
n = nrow(df)
ci = 1.96 * sqrt(rate * (1 - rate) / n)
@printf("\nMC success rate (%s): %.1f%% ± %.1f%% (N = %d)\n",
    controller_config, 100rate, 100ci, n)

# Conditional failure rates: split each parameter at its median, compare
# failure rate in the upper vs lower half to rank influence.
println("\nPer-parameter conditional failure rates (upper vs lower half):")
for par in (:mp, :Lp, :mr, :kt, :km, :Rm, :Jr, :Jp, :r_cm_r, :br, :bp, :tau_c_sh, :tau_c_el)
    hasproperty(df, par) || continue
    v = df[!, par]
    all(ismissing, v) && continue
    med = median(skipmissing(v))
    up = mean(.!df.success[coalesce.(v .> med, false)])
    lo = mean(.!df.success[coalesce.(v .<= med, false)])
    @printf("  %-9s  fail(low) %5.1f%%  fail(high) %5.1f%%\n", par, 100lo, 100up)
end
