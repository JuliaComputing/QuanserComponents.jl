#=
This script runs the swing-up controller on the physical Furuta pendulum. The
controller is the generated synchronous program `QuanserComponents.SwingupController`,
which contains the complete state machine: it first homes the arm (`GoHome`), then
switches to energy-based swing-up plus LQR stabilization wrapped by error recovery
(`RuntimeController`), and re-homes if the arm stays out of bounds. All homing,
saturation and out-of-bounds recovery therefore live inside the controller; the
hardware loop just measures, calls the controller, and applies the returned voltage.
=#
using QuanserInterface
using QuanserInterface.HardwareAbstractions
using QuanserComponents
# using ControlSystemsBase
using QuanserInterface: energy, measure
using StaticArrays
using Plots


const rr = Ref([0, pi, 0, 0])
nu  = 1     # number of controls
nx  = 4     # number of states
Ts  = 0.005 # sampling time
ctrl = QuanserComponents.SwingupController(; Ts, backend=:julia)


using Statistics
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
    plot!(diff(D[1,:]), sp=4, lab="Δt"); hline!([process.Ts], sp=4, framestyle=:zerolines, lab="Ts")
end

function swingup(process; Tf = 10, verbose=true)
    Ts = process.Ts
    N = round(Int, Tf/Ts)
    data = Vector{Vector{Float64}}(undef, 0)
    sizehint!(data, N)

    simulation = processtype(process) isa SimulatedProcess

    # Reset the controller's internal state so every run starts in the homing
    # state of the state machine.
    QuanserComponents.SynchToolkit.reset!(ctrl)

    y = QuanserInterface.measure(process)
    if verbose && !simulation
        @info "Starting experiment from y: $y"
    end

    try
        # GC.gc()
        GC.enable(false)
        t_start = time()
        u = 0.0
        for i = 1:N
            HardwareAbstractions.@periodically Ts begin
                t = simulation ? (i-1)*Ts : time() - t_start
                y = QuanserInterface.measure(process)
                # The synchronous program handles homing, swing-up, stabilization
                # and out-of-bounds recovery internally.
                u = ctrl(y[1], y[2])
                control(process, [u])
                verbose && @info "t = $(round(t, digits=3)), u = $(round(u, digits=3))"
                push!(data, [t; y; u])
            end
        end
    catch e
        @error "Terminating" e
        # rethrow()
    finally
        control(process, [0.0])
        GC.enable(true)
        # GC.gc()
    end

    reduce(hcat, data)
end
##
process = QuanserInterface.QubeServoPendulum(; Ts)
home!(process, 40)
##
function runplot(process; kwargs...)
    rr[][1] = deg2rad(0)
    rr[][2] = pi
    y = QuanserInterface.measure(process)
    # if processtype(process) isa SimulatedProcess
    #     process.x = 0*process.x
    # elseif abs(y[2]) > 0.8 || !(-2.5 < y[1] < 2.5)
    #     @info "Auto homing"
    #     autohome!(process)
    # end
    global D
    D = swingup(process; kwargs...)
    plotD(D)
end

runplot(process; Tf = 10)

using DelimitedFiles
writedlm("swingup.csv", permutedims([["time", "shoulder_angle", "elbow_angle", "control_input"] D]))


# ## Simulated process
# process = QuanserInterface.QubeServoPendulumSimulator(; Ts, p = QuanserInterface.pendulum_parameters(true));

# @profview_allocs runplot(process; Tf = 5) sample_rate=0.1

# ##

# task = @spawn runplot(process; Tf = 15)
# rr[][1] = deg2rad(-30)
# rr[][1] = deg2rad(-20)
# rr[][1] = deg2rad(-10)
# rr[][1] = deg2rad(0)
# rr[][1] = deg2rad(10)
# rr[][1] = deg2rad(20)
# rr[][1] = deg2rad(30)


# rr[][2] = pi
# rr[][2] = 0