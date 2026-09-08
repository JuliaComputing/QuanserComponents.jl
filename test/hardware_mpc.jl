#=
This script runs the MPC controller on the physical Furuta pendulum. The controller is the
`FurutaMPCHardware` model compiled to a synchronous program with `compile_program`: an
`MPCComponents.ACADOSMPC` that swings the pendulum up and balances it -- a
nonlinear MPC whose prediction model is the multibody `QubePendulum` itself, differentiated
with ForwardDiff (the `jacobian_backend = ForwardDiff` option of `ACADOSMPC`), with the motor
voltage and the arm angle constrained.

Like the swing-up program, this one does its own I/O and its own logging, so there is
nothing for this script to do but hand the program to `run_inprocess!`. The log has the
swing-up log's first six columns plus acados' `exitflag` of every solve.

The second part runs the same model against the device as a simulation instead
(`run_ode!`: the model paces itself in real time and records the MPC's
predicted trajectories and solver residuals) and opens `MPCComponents.mpc_gui` on the
result -- one panel per state and control with the history up to a slider time and the
prediction from that tick, and a solver panel. That needs GLMakie in the environment.

There is no homing: the arm starts wherever it is. Before starting, let the pendulum hang
straight down and pass how far the arm is from centre as `arm_deg`.

ENVIRONMENT: the repository checks in a Manifest.toml that resolves the whole stack, with
MultibodyComponents expected as `../MultibodyComponents` next to this repository; see the
"Nonlinear MPC" section of the README. Then

  julia --project=. test/hardware_mpc.jl
=#

using QuanserComponents
using QuanserComponents: compile_program, ProgramRuntime, run_inprocess!, run_ode!, program_spec,
                         read_log, hardware_counters, build_qube_hw!, have_hil, MPC_LOG_COLUMNS
using ModelingToolkit: mtkcompile, ODEProblem
using SynchToolkit
using Printf
using Statistics
using Plots

Ts = 0.01
logfile = "run_mpc.csv"

have_hil() || build_qube_hw!(; hil = true, force = true)

# Compiling the model (the multibody plant twice over: once as the prediction model, once
# as the acados solver's model) takes a while; keep the compiled program around between runs.
# `Np` is the horizon in samples, `umax` the MPC's voltage bound; the MPC's weights are
# set with Dyad override paths, e.g. `control_system__Q1 = diagm([100.0, 100.0, 1.0, 1.0])`.
# `command_umax` clamps the command before the amplifier and can be changed without a
# recompile (`ProgramRuntime(gen; command_umax = 5.0)`) -- a first run at reduced voltage.
@time "compile FurutaMPCHardware" gen = compile_program(FurutaMPCHardware; Ts, Np = 60,
                                                        log_file = logfile)
ctrl = ProgramRuntime(gen)

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
# `run_inprocess!` switches the garbage collector off for the run so no collection lands inside
# a 5 ms period; this program allocates a few MB per tick in acados' Julia callbacks, which a
# 10 s run can afford. For a long run pass `disable_gc = false`.
r = run_inprocess!(ctrl; Tf = 10, arm_deg = 0)

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
# The MPC's own verdict: every solve should report status 0 (2 is the iteration limit), and
# the execution time is the number to compare with the clock period.
@printf("exitflag != 0 on %d of %d ticks (status 2 on %d); exec: median %.2f ms, 99%% %.2f ms, max %.2f ms\n",
        count(!=(0), log.exitflag), length(log.exitflag), count(==(2), log.exitflag),
        1e3median(log.exec), 1e3quantile(log.exec, 0.99), 1e3maximum(log.exec))

cnt = hardware_counters()
@info "hardware calls" cnt.n_measure cnt.n_write ticks=r.ticks rows=r.rows log=r.log_file

# --- the MPC debug GUI ---------------------------------------------------------------------
# The same model, run against the device as a simulation: an ODE solver steps the clocked
# partition, `HardwareDiagnostics(realtime = true)` holds each tick to the wall clock, and the
# MPC records what it predicted at every tick (`output_trajectories = true`). Pendulum hanging,
# arm where it is, as above. The model and the problem are built here so that they can be kept
# between runs; `spec.ode_kwargs` is `realtime = true, output_trajectories = true`.
using GLMakie
using MPCComponents: mpc_gui
spec = program_spec(FurutaMPCHardware)
gui_log = "run_mpc_gui.csv"
@time "compile the model" begin
    model = FurutaMPCHardware(; name = spec.name, Ts, Np = 60, log_file = gui_log, spec.ode_kwargs...)
    ssys = mtkcompile(model; additional_passes = [SynchToolkit.compile_lustre])
    prob = ODEProblem(ssys, Pair[], (0.0, 3.0); build_initializeprob = false)
end
@time "run" sol = run_ode!(FurutaMPCHardware, prob; arm_deg = 0, log_file = gui_log)
fig, tslider = mpc_gui(model, sol)   # drag the slider, or set tslider[] = 2.0
display(fig)
# The solution also carries the pacing diagnostics: how late each tick was, in seconds.
late = sol[ssys.diagnostics.late]
@printf("simulated run: %d ticks, late ticks %d, max lateness %.2f ms\n",
        length(late), count(>(0), late), 1e3maximum(late))
