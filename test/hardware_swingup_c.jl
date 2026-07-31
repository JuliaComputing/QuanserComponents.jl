# The exported binary talks to the board itself through qube_hw.c, so nothing here
# needs QuanserInterface: this script only exports, builds, runs and plots.
using QuanserComponents
using DelimitedFiles
using StaticArrays
using Plots
using Statistics

Ts  = 0.005 # sampling time

function centraldiff(v::AbstractMatrix)
    dv = Base.diff(v, dims=1)/2
    a1 = [dv[[1],:];dv]
    a2 = [dv;dv[[end],:]]
    a = a1+a2
end

function centraldiff(v::AbstractVector)
    dv = Base.diff(v)/2
    a1 = [dv[1];dv]
    a2 = [dv;dv[end]]
    a = a1+a2
end
function plotD(D, th=0.2)
    if size(D, 2) > 200*200
        return
    end
    tvec = D[1, :]
    y = D[2:3, :]'
    # y[:, 2] .-= pi
    # y[:, 2] .*= -1
    u = D[4, :]
    # plot(tvec, xh, layout=6, lab=["arm" "pend" "arm ω" "pend ω"] .* " estimate", framestyle=:zerolines)
    plot(tvec, y, sp=[1 2], lab = ["arm" "pend"] .* " meas", framestyle=:zerolines, layout=4)
    hline!([-pi pi], lab="", sp=2)
    hline!([-pi-th -pi+th pi-th pi+th], lab="", l=(:black, :dash), sp=2)
    # plot!(tvec, centraldiff(y) ./ median(diff(tvec)), sp=[3 4], lab="central diff")
    plot!(tvec, u, sp=3, lab = "u", framestyle=:zerolines)
    plot!(diff(D[1,:]), sp=4, lab="Δt"); hline!([Ts], sp=4, framestyle=:zerolines, lab="Ts")
end

dir = joinpath(@__DIR__, "..", "furuta_c")

##
# Export, build, run on the hardware, and bring up kst2 alongside the run. The analysis
# launches the viewer with its working directory set to `output_dir`, so the session file's
# relative reference to run_hardware.csv resolves without changing this process's cwd.
@time sol = QuanserComponents.FurutaSwingupExperiment(; output_dir = dir, Ts, run = true,
                                              live_plot = true)

# The log was written by the program itself, from inside each tick, and fetched back here if
# the run happened on the deploy host. `sol.hwrun.log` is where it landed.
D = readdlm(sol.hwrun.log, skipstart=1)'
plotD(D)
# The viewer outlives the analysis so the trace stays up; close it with:
# kill(sol.hwrun.plotter)
##

##
# Export only, without touching the hardware:
# QuanserComponents.FurutaSwingupExperiment(; output_dir = dir, Ts, run = false)
##