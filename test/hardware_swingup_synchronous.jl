#=
This script runs the swing-up controller on the physical Furuta pendulum with the
hardware I/O *inside* the synchronous program. It is the counterpart of
`hardware_swingup.jl`: there, the loop measures, calls the controller and applies
the voltage; here the compiled program `QuanserComponents.HardwareSwingupController`
(the `FurutaHardware` model) does all three itself, so the loop only keeps time and
records what the program reports back.

The controller is otherwise the same `SwingupWithHoming` state machine, so homing,
saturation and out-of-bounds recovery still live inside the controller. The encoder
read is done by `HardwareMeasurement` and the amplifier write by `HardwareCommand`,
both bound to this process through `bind_hardware!`.

ENVIRONMENT: run in the package `test/` environment (it has QuanserInterface):
  julia --project=test test/hardware_swingup_synchronous.jl
=#

using QuanserComponents
using QuanserComponents: HardwareIO, bind_hardware!, HardwareSwingupController
using QuanserInterface
using QuanserInterface: control, measure, home!, processtype, SimulatedProcess
using HardwareAbstractions
using SynchToolkit
using DelimitedFiles
using Printf
using Statistics
using Plots

Ts = 0.005

# Compiling the model takes a while; keep the controller around between runs.
@time "compile FurutaHardware" ctrl = HardwareSwingupController(; Ts)

"""
    swingup(process, ctrl; Tf = 10, verbose = true) -> Matrix

Run the controller for `Tf` seconds. Returns a 4 x N matrix
`[t; shoulder; elbow; u]` built from what the program measured and applied,
truncated if the loop was cut short.
"""
function swingup(process, ctrl; Tf = 10, verbose = true)
    N = round(Int, Tf / Ts)
    log = Matrix{Float64}(undef, 4, N)   # preallocated: GC is disabled below
    n_written = 0

    # Reset both the node's state machine (back to homing) and the I/O cache.
    SynchToolkit.reset!(ctrl)
    verbose && @info "Starting experiment from y: $(measure(process))"

    try
        GC.enable(false)
        t_start = time()
        for i in 1:N
            HardwareAbstractions.@periodically Ts begin
                # One call: reads both encoders, runs the state machine, writes the
                # voltage, and reports all three back.
                out = ctrl()
                log[1, i] = time() - t_start
                log[2, i] = out.shoulder
                log[3, i] = out.elbow
                log[4, i] = out.u
                n_written = i
            end
        end
    catch e
        @error "Terminating" e
    finally
        # The program cannot unwind a write it has already made, so make sure the
        # motor ends up at zero whatever happened.
        control(process, [0.0])
        GC.enable(true)
    end
    log[:, 1:n_written]
end

function plotD(D, th = 0.2)
    size(D, 2) > 200 * 200 && return
    tvec = D[1, :]
    plot(tvec, D[2:3, :]', sp = [1 2], lab = ["arm" "pend"] .* " meas",
         framestyle = :zerolines, layout = 4)
    hline!([-pi pi], lab = "", sp = 2)
    hline!([-pi - th -pi + th pi - th pi + th], lab = "", l = (:black, :dash), sp = 2)
    plot!(tvec, D[4, :], sp = 3, lab = "u applied", framestyle = :zerolines)
    plot!(diff(tvec), sp = 4, lab = "Δt")
    hline!([Ts], sp = 4, framestyle = :zerolines, lab = "Ts")
end

# --- main --------------------------------------------------------------------
process = QuanserInterface.QubeServoPendulum(; Ts)
home!(process, -5)

bind_hardware!(ctrl; measure = () -> measure(process),
                     control = u -> control(process, [u]))

D = swingup(process, ctrl; Tf = 10)
plotD(D)

@printf("%d samples, %.1f s, arm in [%.1f, %.1f] deg, pendulum in [%.1f, %.1f] deg, |u| <= %.2f V\n",
        size(D, 2), size(D, 2) * Ts,
        rad2deg(minimum(D[2, :])), rad2deg(maximum(D[2, :])),
        rad2deg(minimum(D[3, :])), rad2deg(maximum(D[3, :])), maximum(abs, D[4, :]))
@printf("loop timing: median dt %.4f s, max %.4f s\n",
        median(diff(D[1, :])), maximum(diff(D[1, :])))

# The I/O counters must match the number of ticks -- one read and one write each.
@info "hardware calls" n_measure=ctrl.io.n_measure n_control=ctrl.io.n_control ticks=size(D, 2)

writedlm("swingup_synchronous.csv",
         permutedims([["time", "shoulder_angle", "elbow_angle", "control_input"] D]))
