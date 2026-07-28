#=
This script runs the swing-up controller on the physical Furuta pendulum. The
controller is the generated synchronous program `QuanserComponents.SwingupController`
(the `FurutaHardware` model), which contains the complete state machine: it first
homes the arm (`GoHome`), then switches to energy-based swing-up plus LQR
stabilization wrapped by error recovery (`RuntimeController`), and re-homes if the
arm stays out of bounds.

The controller also does its own I/O. `HardwareMeasurement` reads the encoders and
`HardwareCommand` writes the motor voltage, both by calling into csrc/qube_hw.c --
the same C implementation the exported standalone binary links against. So this
loop only keeps time and records what the controller reports back; there is no
`measure`/`control` call here, and no QuanserInterface in the loop.

Before starting, let the pendulum hang straight down; `open_hardware!` records the
current encoder counts as the homing offsets. The arm does not have to be at its
home position -- pass `arm_deg` to say how far off it is, as with
`QuanserInterface.home!(process, -5)`.

ENVIRONMENT: run in the package `test/` environment:
  julia --project=test test/hardware_swingup.jl
=#

using QuanserComponents
using QuanserComponents: SwingupController, open_hardware!, close_hardware!,
                         hardware_counters, build_qube_hw!, have_hil
using HardwareAbstractions
using SynchToolkit
using DelimitedFiles
using Printf
using Statistics
using Plots

Ts = 0.005

# Needs the Quanser HIL SDK linked in; `build_qube_hw!` defaults to enabling it when
# the SDK is installed, so this only matters if the library was built without it.
have_hil() || build_qube_hw!(; hil = true, force = true)

# Compiling the model takes a while; keep the controller around between runs.
@time "compile FurutaHardware" ctrl = SwingupController(; Ts, backend = :julia)
# For the C backend instead (identical control signal, ~the same speed here since
# the hot path is the same C either way):
# ctrl = SwingupController(; Ts, backend = :c)

"""
    swingup(ctrl; Tf = 10) -> Matrix

Run the controller for `Tf` seconds. Returns a 4 x N matrix `[t; shoulder; elbow; u]`
built from what the program measured and applied, truncated if cut short.
"""
function swingup(ctrl; Tf = 10)
    N = round(Int, Tf / Ts)
    log = Matrix{Float64}(undef, 4, N)   # preallocated: GC is disabled below
    n_written = 0

    # Back to the homing state, and clear the I/O counters.
    SynchToolkit.reset!(ctrl)

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
        close_hardware!()
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
# Pendulum hanging, arm parked wherever is convenient: `arm_deg` is where the arm
# physically is now (degrees), so 0 still means centred and there is no need to nudge
# the arm into position by hand. Same calibration as `home!(process, -5)`.
open_hardware!(:hil; arm_deg = -5)

D = swingup(ctrl; Tf = 10)
plotD(D)

@printf("%d samples, %.1f s, arm in [%.1f, %.1f] deg, pendulum in [%.1f, %.1f] deg, |u| <= %.2f V\n",
        size(D, 2), size(D, 2) * Ts,
        rad2deg(minimum(D[2, :])), rad2deg(maximum(D[2, :])),
        rad2deg(minimum(D[3, :])), rad2deg(maximum(D[3, :])), maximum(abs, D[4, :]))
@printf("loop timing: median dt %.4f s, max %.4f s\n",
        median(diff(D[1, :])), maximum(diff(D[1, :])))

# One read and one write per tick, or the loop dropped/duplicated a sample.
cnt = hardware_counters()
@info "hardware calls" cnt.n_measure cnt.n_write ticks=size(D, 2)

# writedlm("swingup.csv",
#          permutedims([["time", "shoulder_angle", "elbow_angle", "control_input"] D]))
