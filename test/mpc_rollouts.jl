#=
Monte Carlo verification of the MPC swing-up controller (`FurutaMPCHardware`) in
simulation: the compiled synchronous program -- the very node that runs on the rig -- is
ticked against a simulated Furuta pendulum from about a thousand random initial
conditions, and every rollout is checked for a swing-up that holds within 10 s.

The plant simulator is the multibody `QubePendulum` ODE (the same model the MPC predicts
with, taken from `furuta_mpc_dynamics()`), integrated with RK4 at five sub-steps per
controller period, with the encoder quantization of the QUBE (2048 counts per revolution)
applied to the measured angles. The program reads the simulator through `bind_hardware!`,
exactly as the tests do, so the velocity estimators, the angle wrapping, the swing-up /
MPC switch and the MPC itself all run as compiled.

Initial conditions are drawn uniformly from the operating space
    arm angle        ±1.5 rad         (inside the ±1.92 rad end stops)
    pendulum angle   [0, 2π)          (anywhere, including near upright)
    arm velocity     ±3 rad/s
    pendulum velocity ±10 rad/s
A rollout succeeds if the pendulum is within 0.1 rad of upright for the whole last second
of the 10 s run; the catch time is the instant from which it stays there. The arm's
excursion is compared with the end stops as well, since that is the MPC's constraint.

Writes the plots and a summary to `outdir` (default `mpc_rollouts/` in the current directory)
and prints the statistics. Runtime is dominated by the MPC solves: about a millisecond per
tick, some 2000 ticks per rollout, so roughly half an hour for the default 1000 rollouts.

ENVIRONMENT: as for test/hardware_mpc.jl (see the README's "Nonlinear MPC" section):
  julia --project=<env> test/mpc_rollouts.jl [nrollouts] [outdir]
=#

using QuanserComponents
import QuanserComponents as QC
using Statistics, Random, Printf
using Plots

const Ts = 0.005
const Tf = 10.0
const NROLL = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 1000
const OUTDIR = length(ARGS) >= 2 ? ARGS[2] : "mpc_rollouts"
const ARM_LIMIT = 1.9198621771937625        # FurutaMPC's default arm_limit (110 deg)
const CATCH_TOL = 0.1                       # rad from upright counted as balanced
const HOLD = 1.0                            # s the pendulum must stay balanced at the end
mkpath(OUTDIR)

# ---------------------------------------------------------------------------
## Plant simulator
# ---------------------------------------------------------------------------
struct FurutaSim
    f!::Function
    p::Vector{Float64}
    x::Vector{Float64}
    Ts::Float64
    nsub::Int
    quantize::Bool
    k::Vector{Float64}
end
FurutaSim(dyn; Ts = Ts, nsub = 5, quantize = true) =
    FurutaSim(dyn.f!, copy(dyn.p_default), zeros(4), Ts, nsub, quantize, zeros(20))

const ENC_RES = 2pi / 2048
quant(x) = round(x / ENC_RES) * ENC_RES

function step!(s::FurutaSim, u)
    h = s.Ts / s.nsub
    x = s.x
    k1 = view(s.k, 1:4); k2 = view(s.k, 5:8); k3 = view(s.k, 9:12); k4 = view(s.k, 13:16); xt = view(s.k, 17:20)
    uu = [u]
    for _ in 1:s.nsub
        s.f!(k1, x, uu, s.p, 0.0)
        @. xt = x + h / 2 * k1; s.f!(k2, xt, uu, s.p, 0.0)
        @. xt = x + h / 2 * k2; s.f!(k3, xt, uu, s.p, 0.0)
        @. xt = x + h * k3;     s.f!(k4, xt, uu, s.p, 0.0)
        @. x += h / 6 * (k1 + 2k2 + 2k3 + k4)
    end
    return x
end
measure(s::FurutaSim) = s.quantize ? (quant(s.x[1]), quant(s.x[2])) : (s.x[1], s.x[2])

"Tick the program against the simulator from `x0` for `Tf` seconds."
function rollout!(ctrl, s::FurutaSim, x0; Tf = Tf)
    s.x .= x0
    applied = Ref(0.0)
    QC.bind_hardware!(measure = () -> measure(s), control = u -> (applied[] = u))
    QC.SynchToolkit.reset!(ctrl)
    N = round(Int, Tf / s.Ts)
    X = zeros(4, N); U = zeros(N); flag = zeros(N); stab = falses(N); texec = zeros(N)
    for i in 1:N
        t0 = time_ns()
        out = ctrl()
        texec[i] = (time_ns() - t0) / 1e9
        X[:, i] .= s.x; U[i] = applied[]; flag[i] = out.exitflag; stab[i] = out.stabilizing > 0.5
        step!(s, applied[])
    end
    return (; X, U, flag, stab, texec)
end

"Time from which the pendulum stays within `tol` of upright to the end (NaN if it does not hold for `hold` s)."
function catch_time(X; tol = CATCH_TOL, hold = HOLD)
    up = abs.(mod.(X[2, :], 2pi) .- pi) .< tol
    N = length(up); i = N
    while i >= 1 && up[i]; i -= 1; end
    return N - i >= round(Int, hold / Ts) ? i * Ts : NaN
end

# ---------------------------------------------------------------------------
## Run
# ---------------------------------------------------------------------------
@time "prediction model" dyn = QC.furuta_mpc_dynamics()
@time "compile FurutaMPCHardware" ctrl = QC.MPCController(; Ts)
sim = FurutaSim(dyn; Ts)

rng = Xoshiro(1)
sample_x0(rng) = [1.5 * (2rand(rng) - 1), 2pi * rand(rng), 3.0 * (2rand(rng) - 1), 10.0 * (2rand(rng) - 1)]
x0s = [sample_x0(rng) for _ in 1:NROLL]

tcatch = fill(NaN, NROLL); armmax = zeros(NROLL); umax = zeros(NROLL)
nbadflag = zeros(Int, NROLL); nmpc = zeros(Int, NROLL)
exec_mpc = Float64[]; exec_swing = Float64[]
NKEEP = min(NROLL, 100)                    # trajectories kept for the overlay plot
kept = Vector{Any}(undef, NKEEP)
twall = @elapsed for (i, x0) in enumerate(x0s)
    r = rollout!(ctrl, sim, x0)
    tcatch[i] = catch_time(r.X)
    armmax[i] = maximum(abs, r.X[1, :])
    umax[i] = maximum(abs, r.U)
    nbadflag[i] = count(!=(0), r.flag[r.stab]); nmpc[i] = count(r.stab)
    append!(exec_mpc, r.texec[r.stab]); append!(exec_swing, r.texec[.!r.stab])
    i <= NKEEP && (kept[i] = r)
    i % 50 == 0 && @printf("%4d/%d rollouts, %d successes so far\n", i, NROLL, count(!isnan, tcatch[1:i]))
end

success = .!isnan.(tcatch)
summary = @sprintf("""
    MPC swing-up Monte Carlo: %d rollouts of %.0f s, Ts = %.3f s, Np = 30
    successes (upright within %.2f rad for the last %.1f s): %d / %d = %.1f%%
    catch time [s]: median %.2f, 90%% %.2f, max %.2f
    arm excursion max |phi| [rad]: median %.2f, max %.2f (end stops at %.2f); rollouts beyond the stops: %d
    |u| max over all rollouts: %.2f V
    acados exitflag != 0 on %d of %d MPC ticks
    tick execution time, MPC in command  [ms]: median %.3f, 99%% %.3f, max %.3f
    tick execution time, energy swing-up [ms]: median %.3f, 99%% %.3f, max %.3f
    wall time %.1f min (%.2f ms per tick incl. the plant simulation)
    """, NROLL, Tf, Ts, CATCH_TOL, HOLD, count(success), NROLL, 100mean(success),
    median(tcatch[success]), quantile(tcatch[success], 0.9), maximum(tcatch[success]),
    median(armmax), maximum(armmax), ARM_LIMIT, count(>(ARM_LIMIT), armmax), maximum(umax),
    sum(nbadflag), sum(nmpc),
    1e3median(exec_mpc), 1e3quantile(exec_mpc, 0.99), 1e3maximum(exec_mpc),
    1e3median(exec_swing), 1e3quantile(exec_swing, 0.99), 1e3maximum(exec_swing),
    twall / 60, 1e3twall / (NROLL * Tf / Ts))
println(summary)
write(joinpath(OUTDIR, "summary.txt"), summary)
open(joinpath(OUTDIR, "rollouts.csv"), "w") do io
    println(io, "shoulder0\telbow0\tshoulder_vel0\telbow_vel0\tcatch_time\tarm_max\tu_max\tbad_exitflags\tmpc_ticks")
    for i in 1:NROLL
        println(io, join(string.([x0s[i]; tcatch[i]; armmax[i]; umax[i]; nbadflag[i]; nmpc[i]]), '\t'))
    end
end

# ---------------------------------------------------------------------------
## Plots
# ---------------------------------------------------------------------------
default(; fontfamily = "sans-serif", linewidth = 1.5, grid = true, gridalpha = 0.15,
        framestyle = :box, legend = :topright, dpi = 150, size = (900, 500))
BLUE = "#2166ac"; ORANGE = "#e08214"; GRAY = "#8a8a8a"
tvec = (0:round(Int, Tf / Ts)-1) .* Ts

# 1. Every kept rollout's distance from upright, and the arm angle, over time.
p1 = plot(layout = (2, 1), size = (900, 650), link = :x)
for r in kept
    plot!(p1[1], tvec, abs.(mod.(r.X[2, :], 2pi) .- pi), color = BLUE, alpha = 0.25, lw = 0.8, label = "")
    plot!(p1[2], tvec, r.X[1, :], color = BLUE, alpha = 0.25, lw = 0.8, label = "")
end
hline!(p1[1], [CATCH_TOL], color = GRAY, ls = :dash, label = "balanced band")
plot!(p1[1], ylabel = "pendulum angle from upright [rad]", yscale = :log10, ylims = (1e-4, 4),
      title = "First $(NKEEP) rollouts: pendulum and arm")
hline!(p1[2], [-ARM_LIMIT, ARM_LIMIT], color = GRAY, ls = :dash, label = "end stops")
plot!(p1[2], ylabel = "arm angle [rad]", xlabel = "time [s]", ylims = (-2.2, 2.2))
savefig(p1, joinpath(OUTDIR, "trajectories.png"))

# 2. Catch-time distribution.
p2 = histogram(tcatch[success], bins = 0:0.25:Tf, color = BLUE, linecolor = :white,
               label = "", xlabel = "catch time [s]", ylabel = "rollouts",
               title = @sprintf("Time to swing up and stay balanced (%d of %d succeeded)", count(success), NROLL))
savefig(p2, joinpath(OUTDIR, "catch_times.png"))

# 3. Where in the initial-condition space the slow (or failed) swing-ups are.
ct_plot = replace(tcatch, NaN => Tf)
p3 = scatter([x[2] for x in x0s], [x[1] for x in x0s], zcolor = ct_plot, color = :blues,
             markerstrokewidth = 0, markersize = 4, label = "", colorbar_title = "catch time [s]",
             xlabel = "initial pendulum angle [rad] (π is upright)", ylabel = "initial arm angle [rad]",
             title = "Catch time over the initial conditions", xticks = ([0, pi/2, pi, 3pi/2, 2pi], ["0", "π/2", "π", "3π/2", "2π"]))
any(!, success) && scatter!(p3, [x[2] for x in x0s[.!success]], [x[1] for x in x0s[.!success]],
                            color = ORANGE, markersize = 6, markershape = :x, label = "failed")
savefig(p3, joinpath(OUTDIR, "initial_conditions.png"))

# 4. Per-tick execution time, MPC in command versus swing-up.
p4 = histogram(1e3exec_swing, bins = 0:0.02:3, color = GRAY, linecolor = :white, alpha = 0.8,
               label = "energy swing-up", xlabel = "controller execution time per tick [ms]", ylabel = "ticks",
               title = "Tick execution time (clock period 5 ms)", yscale = :log10)
histogram!(p4, 1e3exec_mpc, bins = 0:0.02:3, color = BLUE, linecolor = :white, alpha = 0.8, label = "MPC in command")
savefig(p4, joinpath(OUTDIR, "exec_times.png"))

# 5. Arm excursion against the end stops.
p5 = histogram(armmax, bins = 0:0.05:2.2, color = BLUE, linecolor = :white, label = "",
               xlabel = "largest |arm angle| during the rollout [rad]", ylabel = "rollouts",
               title = "Arm excursion versus the end stops")
vline!(p5, [ARM_LIMIT], color = ORANGE, ls = :dash, lw = 2, label = "end stops")
savefig(p5, joinpath(OUTDIR, "arm_excursion.png"))
println("plots written to ", abspath(OUTDIR))
