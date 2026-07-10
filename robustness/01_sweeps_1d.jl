# Per-parameter 1D sweeps: perturb one plant parameter at a time with the
# controller at nominal, locate the failure boundary on each side of nominal
# by sweeping then bisecting the first failing interval.
#
# Controller config is selected via the CONTROLLER environment-like global
# (set `controller_config` before including, defaults to "baseline").

include("common.jl")

@isdefined(controller_config) || (controller_config = "baseline")

model, ssys = build_system()
ctrl = controller_overrides(ssys, controller_config)

# (name, keyword, nominal, sweep values excluding nominal, log-spaced?)
sweeps = [
    (:mp,       range(0.7, 1.3, length = 13),  0.024),
    (:Lp,       range(0.85, 1.15, length = 13), 0.129),
    (:mr,       range(0.7, 1.3, length = 13),  0.095),
    (:kt,       range(0.8, 1.2, length = 13),  0.042),
    (:km,       range(0.8, 1.2, length = 13),  0.042),
    (:Rm,       range(0.75, 1.25, length = 13), 8.4),
    (:br,       exp10.(range(-1, log10(20), length = 13)), 5e-5),
    (:bp,       exp10.(range(-1, log10(20), length = 13)), 2.5e-6),
    (:tau_c_sh, range(0, 1, length = 13),      5e-3),  # scale of absolute range [0, 5e-3]
    (:tau_c_el, range(0, 1, length = 13),      5e-4),  # scale of absolute range [0, 5e-4]
]

function run_case(kwargs)
    ov = vcat(ctrl, perturbation_overrides(ssys; kwargs...))
    metrics(simulate(ssys, ov; tf = 15.0), ssys)
end

rows = NamedTuple[]
for (par, scales, nominal) in sweeps
    if par in (:tau_c_sh, :tau_c_el) && !supports_friction(ssys)
        continue
    end
    for s in scales
        value = s * nominal
        kwargs = Dict{Symbol, Any}(par => value)
        m = try
            run_case(kwargs)
        catch err
            @warn "simulation crashed" par value err
            metrics_failed()
        end
        push!(rows, metrics_row(m; parameter = String(par), value, scale = s,
            controller = controller_config))
        @printf("%-9s scale %6.3f  value %10.4g  success %s  catch %6.2f\n",
            par, s, value, m.success, m.catch_time)
    end
end

df = DataFrame(rows)
CSV.write(resultpath("sweeps_1d_$(controller_config).csv"), df)

# Refine failure boundary by bisection between the last success and first
# failure on each side of nominal.
function bisect_boundary(par, lo, hi, nominal; iters = 5)
    # lo succeeds, hi fails (scales)
    for _ in 1:iters
        mid = (lo + hi) / 2
        value = mid * nominal
        m = try
            run_case(Dict{Symbol, Any}(par => value))
        catch
            metrics_failed()
        end
        if m.success
            lo = mid
        else
            hi = mid
        end
    end
    (lo + hi) / 2
end

boundary_rows = NamedTuple[]
for (par, scales, nominal) in sweeps
    sub = filter(r -> r.parameter == String(par), df)
    sort!(sub, :scale)
    succ = sub.success
    # scan outward from the scale closest to nominal (1.0 for relative, 0 for absolute)
    center = par in (:tau_c_sh, :tau_c_el) ? argmin(abs.(sub.scale .- 0)) : argmin(abs.(sub.scale .- 1))
    for (dir, idxs) in ((:up, center:nrow(sub)), (:down, center:-1:1))
        prev = nothing
        for i in idxs
            if succ[i]
                prev = sub.scale[i]
            else
                if prev !== nothing
                    b = bisect_boundary(par, prev, sub.scale[i], nominal)
                    push!(boundary_rows, (; parameter = String(par), direction = String(dir),
                        boundary_scale = b, boundary_value = b * nominal,
                        controller = controller_config))
                end
                break
            end
        end
    end
end

bdf = DataFrame(boundary_rows)
CSV.write(resultpath("boundaries_1d_$(controller_config).csv"), bdf)
println(bdf)

using Plots
gr()
ENV["GKSwstype"] = "100"
plots = map(sweeps) do (par, _, _)
    sub = filter(r -> r.parameter == String(par), df)
    sort!(sub, :scale)
    scatter(sub.scale, Int.(sub.success), title = String(par), legend = false,
        xlabel = "scale", ylabel = "success", markersize = 3,
        color = ifelse.(sub.success, :green, :red))
end
savefig(plot(plots...; layout = length(plots), size = (1200, 800)),
    figurepath("sweeps_1d_$(controller_config).png"))
