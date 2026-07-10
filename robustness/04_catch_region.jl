# Catch-region (region-of-attraction slice) estimate for the saturated LQR:
# force the switch to always select the LQR by setting the NearTop threshold
# huge, then classify convergence from a grid of initial elbow states near
# upright. Repeated for perturbed plants to check how much the catch region
# shrinks under mismatch. The result informs the choice of NearTop.th.

include("common.jl")

model, ssys = build_system()

alpha_grid = range(-1.2, 1.2, length = 25)  # elbow angle offset from upright
omega_grid = range(-8.0, 8.0, length = 25)  # elbow angular velocity

"Success = settled upright at tf, starting near the top with LQR always active."
function catch_success(ssys, plant_kwargs, dphi, w; tf = 3.0)
    ov = vcat(
        nominal_overrides(ssys),
        supports_friction(ssys) ? perturbation_overrides(ssys; plant_kwargs...) : Pair[],
        Pair[
            ssys.swingup.neartop.th => 1e6
            ssys.qubependulum.elbow_joint.phi => pi + dphi
            ssys.qubependulum.elbow_joint.w => w
        ],
    )
    m = try
        metrics(simulate(ssys, ov; tf), ssys; window = 0.5, th = 1e6)
    catch
        metrics_failed()
    end
    m.success
end

plants = [
    ("nominal", (;)),
    ("mp_plus20", (; mp = 0.024 * 1.2)),
    ("coulomb", (; tau_c_sh = 2e-3, tau_c_el = 2e-4)),
    ("quantized", (; quantized = true)),
]

using Plots
gr()
ENV["GKSwstype"] = "100"

for (name, kwargs) in plants
    name != "nominal" && !supports_friction(ssys) && continue
    S = falses(length(alpha_grid), length(omega_grid))
    for (i, a) in enumerate(alpha_grid), (j, w) in enumerate(omega_grid)
        S[i, j] = catch_success(ssys, kwargs, a, w)
        @printf("%s: α−π=%6.3f ω=%6.2f -> %s\n", name, a, w, S[i, j])
    end
    CSV.write(resultpath("catch_region_$(name).csv"),
        DataFrame(S, string.(round.(omega_grid, sigdigits = 3))))
    hm = heatmap(omega_grid, alpha_grid, S, xlabel = "elbow velocity [rad/s]",
        ylabel = "α − π [rad]", color = [:red, :green], clims = (0, 1),
        colorbar = false, title = "LQR catch region: $name")
    hline!(hm, [-0.4, 0.4], l = (:dash, :black), label = "NearTop.th")
    savefig(hm, figurepath("catch_region_$(name).png"))
end

# Delay variant
model_d, ssys_d = build_system(delayed = true, delay_n = 1)
S = falses(length(alpha_grid), length(omega_grid))
for (i, a) in enumerate(alpha_grid), (j, w) in enumerate(omega_grid)
    S[i, j] = catch_success(ssys_d, (;), a, w)
end
CSV.write(resultpath("catch_region_delay1.csv"),
    DataFrame(S, string.(round.(omega_grid, sigdigits = 3))))
hm = heatmap(omega_grid, alpha_grid, S, xlabel = "elbow velocity [rad/s]",
    ylabel = "α − π [rad]", color = [:red, :green], clims = (0, 1),
    colorbar = false, title = "LQR catch region: delay 1 sample")
hline!(hm, [-0.4, 0.4], l = (:dash, :black), label = "NearTop.th")
savefig(hm, figurepath("catch_region_delay1.png"))
