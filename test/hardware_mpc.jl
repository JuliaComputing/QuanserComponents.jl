#=
This script runs the MPC swing-up controller on the physical Furuta pendulum. The
controller is the generated synchronous program `QuanserComponents.MPCController` (the
`FurutaMPCHardware` model): the energy-based swing-up pumps the pendulum up and, once it
is within `catch_angle` of upright, an `MPCComponents.ACADOSMPC` balances it -- a nonlinear
MPC whose prediction model is the multibody `QubePendulum` itself, differentiated with
ForwardDiff (the `jacobian_backend = ForwardDiff` option of `ACADOSMPC`), with the motor
voltage and the arm angle constrained.

Like `SwingupController`, the program does its own I/O and its own logging, so there is
nothing for this script to do but hand the program to `run_program!`. The log has the
swing-up log's first six columns plus acados' `exitflag` of every solve and whether the
MPC was in command (`stabilizing`).

There is no homing: the arm starts wherever it is. Before starting, let the pendulum hang
straight down and pass how far the arm is from centre as `arm_deg`.

ENVIRONMENT: MPCComponents is unregistered and the AD Jacobian backend lives on its
`feat/acados-ad-jacobian-backend` branch, so the environment has to be assembled by hand;
see the "Nonlinear MPC" section of the README for the recipe. Then

  julia --project=<that env> test/hardware_mpc.jl
=#

using QuanserComponents
using QuanserComponents: MPCController, run_program!, read_log, hardware_counters,
                         build_qube_hw!, have_hil, MPC_LOG_COLUMNS
using Printf
using Statistics
using Plots

Ts = 0.005
logfile = "run_mpc.csv"

have_hil() || build_qube_hw!(; hil = true, force = true)

# Compiling the model (the multibody plant twice over: once as the prediction model, once
# as the acados solver's model) takes a while; keep the controller around between runs.
# `Np` is the horizon in samples, `umax` the MPC's voltage bound; the MPC's weights are
# set with Dyad override paths, e.g. `control_system__Q1 = diagm([10.0, 20.0, 0.1, 0.1])`.
@time "compile FurutaMPCHardware" ctrl = MPCController(; Ts, Np = 30, log_file = logfile)

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
# Pendulum hanging, arm wherever it is: `arm_deg` is where the arm physically is now.
r = run_program!(ctrl; Tf = 10, arm_deg = 0)

log = read_log(r.log_file)
D = permutedims(reduce(hcat, [getproperty(log, Symbol(c)) for c in MPC_LOG_COLUMNS[1:4]]))
plotD(D)

@printf("%d samples, %.1f s, arm in [%.1f, %.1f] deg, pendulum in [%.1f, %.1f] deg, |u| <= %.2f V\n",
        size(D, 2), size(D, 2) * Ts,
        rad2deg(minimum(D[2, :])), rad2deg(maximum(D[2, :])),
        rad2deg(minimum(D[3, :])), rad2deg(maximum(D[3, :])), maximum(abs, D[4, :]))
@printf("loop timing: median dt %.4f s, max %.4f s\n", r.timing.median_dt, r.timing.max_dt)
@printf("as logged:   median dt %.4f s, max %.4f s, max exec %.4f s\n",
        median(log.dt[2:end]), maximum(log.dt[2:end]), maximum(log.exec))
# The MPC's own verdict: every solve should report status 0, and the execution time while
# the MPC is in command is the number to compare with the 5 ms clock.
stab = log.stabilizing .> 0.5
@printf("MPC in command %.1f%% of the run; exitflag != 0 on %d of %d MPC ticks; exec while MPC active: median %.2f ms, max %.2f ms\n",
        100mean(stab), count(!=(0), log.exitflag[stab]), count(stab),
        1e3median(log.exec[stab]), 1e3maximum(log.exec[stab]; init = 0.0))

cnt = hardware_counters()
@info "hardware calls" cnt.n_measure cnt.n_write ticks=r.ticks rows=r.rows log=r.log_file
