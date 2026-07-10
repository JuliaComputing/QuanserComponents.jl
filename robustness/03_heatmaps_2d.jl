# 2D success heatmaps over parameter pairs that interact, plus panels for
# quantization on/off and actuation delay. Requires the Phase 1 model
# (Coulomb friction + quantization-ready samplers).

include("common.jl")

@isdefined(controller_config) || (controller_config = "baseline")
@isdefined(ngrid) || (ngrid = 15)

model, ssys = build_system()
@assert supports_friction(ssys) "Phase 1 model with friction components required"
ctrl = controller_overrides(ssys, controller_config)

function grid_success(par1, vals1, par2, vals2; fixed = (;), sys = ssys, ctrl = ctrl)
    S = falses(length(vals1), length(vals2))
    for (i, v1) in enumerate(vals1), (j, v2) in enumerate(vals2)
        kwargs = merge(NamedTuple(fixed), (; par1 => v1, par2 => v2))
        m = try
            ov = vcat(ctrl, perturbation_overrides(sys; kwargs...))
            metrics(simulate(sys, ov; tf = 15.0), sys)
        catch
            metrics_failed()
        end
        S[i, j] = m.success
        @printf("%s=%.4g %s=%.4g -> %s\n", par1, v1, par2, v2, m.success)
    end
    S
end

using Plots
gr()
ENV["GKSwstype"] = "100"

function save_heatmap(S, vals1, vals2, name1, name2, fname)
    hm = heatmap(vals2, vals1, S, xlabel = String(name2), ylabel = String(name1),
        color = [:red, :green], clims = (0, 1), colorbar = false,
        title = "success: $(name1) vs $(name2)")
    savefig(hm, figurepath(fname))
    CSV.write(resultpath(replace(fname, ".png" => ".csv")),
        DataFrame(S, string.(round.(vals2, sigdigits = 4))))
end

mp_vals = 0.024 .* range(0.7, 1.3, length = ngrid)
Lp_vals = 0.129 .* range(0.85, 1.15, length = ngrid)
tau_vals = range(0, 5e-3, length = ngrid)

S = grid_success(:mp, mp_vals, :tau_c_sh, tau_vals)
save_heatmap(S, mp_vals, tau_vals, :mp, :tau_c_sh, "heatmap_mp_tauc_$(controller_config).png")

S = grid_success(:mp, mp_vals, :Lp, Lp_vals)
save_heatmap(S, mp_vals, Lp_vals, :mp, :Lp, "heatmap_mp_Lp_$(controller_config).png")

# Quantization on: repeat the mp/tau_c_sh grid with quantized encoders
S = grid_success(:mp, mp_vals, :tau_c_sh, tau_vals; fixed = (; quantized = true))
save_heatmap(S, mp_vals, tau_vals, :mp, :tau_c_sh, "heatmap_mp_tauc_quantized_$(controller_config).png")

# Actuation delay panels: rebuild with the delayed model variant
for n in (1, 2)
    model_d, ssys_d = build_system(delayed = true, delay_n = n)
    ctrl_d = controller_overrides(ssys_d, controller_config)
    S = grid_success(:mp, mp_vals, :tau_c_sh, tau_vals; sys = ssys_d, ctrl = ctrl_d)
    save_heatmap(S, mp_vals, tau_vals, :mp, :tau_c_sh,
        "heatmap_mp_tauc_delay$(n)_$(controller_config).png")
end
