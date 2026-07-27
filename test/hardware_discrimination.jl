# =============================================================================
# Run the discriminating input trajectory designed by
# examples/input_design.jl on the physical QUBE.
#
# This is an OPEN-LOOP replay: the designed voltage sequence is written straight
# to the motor. The arm has no restoring force, so a safety supervisor runs on
# top -- see `SAFE_*` below. The design keeps the arm inside +/-70 deg under both
# candidate models; the supervisor exists for the case where reality disagrees
# with both of them.
#
# Output is written in exactly the same 4-column whitespace format as
# swingup.csv, so it is drop-in for examples/pendulum_identification.jl and
# examples/analyze_discrimination.jl.
#
# ENVIRONMENT: run in the package `test/` environment (it has QuanserInterface):
#   julia --project=test test/hardware_discrimination.jl
# =============================================================================

using QuanserComponents
using QuanserInterface
using HardwareAbstractions
using DelimitedFiles
using Printf

Ts       = 0.005
TRAJFILE = joinpath(pkgdir(QuanserComponents), "input_design.csv")
OUTFILE  = joinpath(pkgdir(QuanserComponents), "discrimination_experiment.csv")

# --- safety ------------------------------------------------------------------
UCLAMP    = 3.0             # V, hard clamp on anything we write
SAFE_WARN = deg2rad(95)     # start pulling the arm back
SAFE_ABORT = deg2rad(120)   # give up (soft limit is 110, hard stop 137)
PULLBACK  = 2.0             # V/rad, pull-back gain past SAFE_WARN

"Voltage to actually write: designed value, or a pull-back if the arm strays."
function supervise(u_des, arm)
    u = abs(arm) > SAFE_WARN ? -PULLBACK * arm : u_des
    clamp(u, -UCLAMP, UCLAMP)
end

# --- load the designed trajectory --------------------------------------------
D  = readdlm(TRAJFILE)
useq = Float64.(D[2:end, 2])          # drop header; column 2 is the voltage
N    = length(useq)
@info "loaded trajectory" TRAJFILE N duration_s=N*Ts peak_V=maximum(abs, useq)

"""
    run_experiment(process; useq) -> Matrix

Open-loop replay of `useq` with the safety supervisor active. Returns a 4 x N
matrix `[t; shoulder; elbow; u]` (truncated if aborted early).
"""
function run_experiment(process, useq)
    N = length(useq)
    log = Matrix{Float64}(undef, 4, N)   # preallocated: GC is disabled below
    n_written = 0
    aborted = false
    y = QuanserInterface.measure(process)
    try
        GC.enable(false)
        t_start = time()
        for i in 1:N
            HardwareAbstractions.@periodically Ts begin
                t = time() - t_start
                y = QuanserInterface.measure(process)
                if abs(y[1]) > SAFE_ABORT
                    @error "arm out of bounds, aborting" arm_deg=rad2deg(y[1]) i
                    aborted = true
                    break
                end
                u = supervise(useq[i], y[1])
                QuanserInterface.control(process, [u])
                log[:, i] = [t, y[1], y[2], u]
                n_written = i
            end
        end
    # catch e
        # @error "terminating" e
    finally
        QuanserInterface.control(process, [0.0])
        GC.enable(true)
    end
    aborted && @warn "experiment aborted early" completed_s=n_written*Ts
    log[:, 1:n_written]
end

# --- main --------------------------------------------------------------------
process = QuanserInterface.QubeServoPendulum(; Ts)
home!(process, 1)
QuanserInterface.go_home(process)      # centre the arm, let the pendulum settle

Dlog = run_experiment(process, useq)

open(OUTFILE, "w") do io
    println(io, "time\tshoulder_angle\telbow_angle\tcontrol_input")
    for k in axes(Dlog, 2)
        @printf(io, "%.6f\t%.6f\t%.6f\t%.6f\n", Dlog[1,k], Dlog[2,k], Dlog[3,k], Dlog[4,k])
    end
end

@printf("wrote %s: %d samples, %.1f s, arm in [%.1f, %.1f] deg, pendulum in [%.1f, %.1f] deg\n",
        OUTFILE, size(Dlog, 2), size(Dlog, 2)*Ts,
        rad2deg(minimum(Dlog[2,:])), rad2deg(maximum(Dlog[2,:])),
        rad2deg(minimum(Dlog[3,:])), rad2deg(maximum(Dlog[3,:])))
