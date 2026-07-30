#=
This script runs the swing-up controller on the physical Furuta pendulum. The
controller is the generated synchronous program `QuanserComponents.SwingupController`
(the `FurutaHardware` model), which contains the complete state machine: it first
homes the arm (`GoHome`), then switches to energy-based swing-up plus LQR
stabilization wrapped by error recovery (`RuntimeController`), and re-homes if the
arm stays out of bounds.

The controller also does its own I/O and its own logging. `HardwareMeasurement` reads
the encoders, `HardwareCommand` writes the motor voltage and `DataLogger` appends a row,
all by calling into csrc/qube_hw.c and csrc/qube_log.c -- the same C implementations the
exported standalone binary links against. So there is nothing for this script to do but
name a log file and hand the program to `run_program!`, which opens the device and the
log, keeps time, and closes both; the same function the analyses use, so this runs what
they run.

Before starting, let the pendulum hang straight down; the device is opened with the
current encoder counts as the homing offsets. The arm does not have to be at its home
position -- pass `arm_deg` to say how far off it is, as with
`QuanserInterface.home!(process, -5)`.

ENVIRONMENT: run in the package `test/` environment:
  julia --project=test test/hardware_swingup.jl
=#

using QuanserComponents
using QuanserComponents: SwingupController, run_program!, read_log, hardware_counters,
                         build_qube_hw!, have_hil, SWINGUP_LOG_COLUMNS
using Printf
using Statistics
using Plots

Ts = 0.005
logfile = "swingup.csv"

# Needs the Quanser HIL SDK linked in; `build_qube_hw!` defaults to enabling it when
# the SDK is installed, so this only matters if the library was built without it.
have_hil() || build_qube_hw!(; hil = true, force = true)

# Compiling the model takes a while; keep the controller around between runs.
@time "compile FurutaHardware" ctrl = SwingupController(; Ts, backend = :julia,
                                                        log_file = logfile)
# For the C backend instead (identical control signal, ~the same speed here since
# the hot path is the same C either way):
# ctrl = SwingupController(; Ts, backend = :c, log_file = logfile)

# `D` is the log the *program* wrote, one row per tick, in SWINGUP_LOG_COLUMNS order:
# time, the two angles, the applied voltage, then dt/exec and the raw encoder counts.
# `plotD` takes the 4 x N matrix layout the old logs used, so old CSVs still plot.
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
r = run_program!(ctrl; Tf = 10, arm_deg = -5)

log = read_log(r.log_file)
D = permutedims(reduce(hcat, [getproperty(log, Symbol(c)) for c in SWINGUP_LOG_COLUMNS[1:4]]))
plotD(D)

@printf("%d samples, %.1f s, arm in [%.1f, %.1f] deg, pendulum in [%.1f, %.1f] deg, |u| <= %.2f V\n",
        size(D, 2), size(D, 2) * Ts,
        rad2deg(minimum(D[2, :])), rad2deg(maximum(D[2, :])),
        rad2deg(minimum(D[3, :])), rad2deg(maximum(D[3, :])), maximum(abs, D[4, :]))
# Two timings: what the loop here measured, and what the program logged from inside the
# tick. They should agree -- if they do not, the discrepancy is the logging call itself.
@printf("loop timing: median dt %.4f s, max %.4f s\n", r.timing.median_dt, r.timing.max_dt)
@printf("as logged:   median dt %.4f s, max %.4f s, max exec %.4f s\n",
        median(log.dt[2:end]), maximum(log.dt[2:end]), maximum(log.exec))

# One read and one write per tick, and one row per tick, or the loop dropped a sample.
cnt = hardware_counters()
@info "hardware calls" cnt.n_measure cnt.n_write ticks=r.ticks rows=r.rows log=r.log_file
