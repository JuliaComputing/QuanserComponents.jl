# Compare robustness campaigns across controller configurations: MC success
# rates with confidence intervals, 1D failure-boundary shifts, and a summary
# table. Run after the per-config campaigns (01, 02) have produced their CSVs.

include("common.jl")
using Statistics

configs = String[]
for f in readdir(resultpath(""))
    m = match(r"^mc_(.+)\.csv$", f)
    m === nothing || push!(configs, m.captures[1])
end
isempty(configs) && error("no mc_<config>.csv results found; run 02_montecarlo.jl first")

println("=== Monte Carlo success rates ===")
for c in sort(configs)
    df = CSV.read(resultpath("mc_$c.csv"), DataFrame)
    r = mean(df.success)
    ci = 1.96 * sqrt(r * (1 - r) / nrow(df))
    @printf("%-16s %5.1f%% ± %4.1f%%  (N = %d)\n", c, 100r, 100ci, nrow(df))
end

println("\n=== 1D failure boundaries (scale of nominal) ===")
bconfigs = String[]
for f in readdir(resultpath(""))
    m = match(r"^boundaries_1d_(.+)\.csv$", f)
    m === nothing || push!(bconfigs, m.captures[1])
end
if !isempty(bconfigs)
    dfs = [CSV.read(resultpath("boundaries_1d_$c.csv"), DataFrame) for c in sort(bconfigs)]
    all_params = unique(vcat((df.parameter for df in dfs)...))
    @printf("%-10s %-6s", "parameter", "dir")
    for c in sort(bconfigs)
        @printf(" %14s", c)
    end
    println()
    for p in all_params, dir in ("down", "up")
        @printf("%-10s %-6s", p, dir)
        for df in dfs
            row = filter(r -> r.parameter == p && r.direction == dir, df)
            if nrow(row) == 0
                @printf(" %14s", "-")
            else
                @printf(" %14.3f", row.boundary_scale[1])
            end
        end
        println()
    end
end
