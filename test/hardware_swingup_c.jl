using QuanserInterface
using QuanserInterface.HardwareAbstractions
using QuanserComponents
using QuanserInterface: energy, measure
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

@time sol = QuanserComponents.FurutaExportC(; run = true)
##
tr = @async begin
    cd(joinpath(@__DIR__, "..", "furuta_c"))
    run(`make`)
    run(`./run_hardware`)
    cd(joinpath(@__DIR__, ".."))
end
sleep(0.1)
tks = @async run(`kst2 kast2config.kst`)

wait(tr)

D = readdlm("furuta_c/run_hardware.csv", skipstart=1)'
plotD(D)
##