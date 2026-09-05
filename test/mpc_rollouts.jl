#=
Monte Carlo verification of the MPC controller (`FurutaMPCHardware`) in simulation: the
compiled synchronous program -- the very node that runs on the rig -- is ticked against a
simulated Furuta pendulum from about a thousand random initial conditions, and every rollout is
checked for a swing-up that holds within 10 s.

The plant simulator is the multibody `QubePendulum` ODE (the same model the MPC predicts
with, taken from `furuta_mpc_dynamics()`), integrated with RK4 at five sub-steps per
controller period, with the encoder quantization of the QUBE (2048 counts per revolution)
applied to the measured angles. The program reads the simulator through `bind_hardware!`,
exactly as the tests do, so the velocity estimators, the angle wrapping and the MPC itself all
run as compiled.

Initial conditions are drawn uniformly from the operating space
    arm angle        ±1.5 rad         (inside the ±1.92 rad end stops)
    pendulum angle   [0, 2π)          (anywhere, including near upright)
    arm velocity     ±3 rad/s
    pendulum velocity ±10 rad/s
A rollout succeeds if the pendulum is within 0.1 rad of upright for the whole last second
of the 10 s run; the catch time is the instant from which it stays there. The arm's
excursion is compared with the end stops as well, since that is the MPC's constraint.

The plant is the MPC's own model by default. `plant = perturbed` simulates the identified
model with the motor constant 15 % lower, the arm 20 % heavier, the pendulum inertia 15 %
larger and twice the pendulum damping, while the MPC keeps predicting with the identified
parameters -- a model-mismatch check in the direction the hardware will exercise. `plant =
nominal` uses the datasheet set, which is far off (a third of the identified acceleration per
volt) and is not expected to work.

Writes the plots and a summary to `outdir` (default `mpc_rollouts/` in the current directory)
and prints the statistics. Runtime is dominated by the MPC solves (SQP over a 60-interval
horizon), a few milliseconds per tick, 1000 ticks per rollout.

ENVIRONMENT: as for test/hardware_mpc.jl (see the README's "Nonlinear MPC" section):
  julia --project=<env> test/mpc_rollouts.jl [nrollouts] [outdir] [plant = identified | perturbed | nominal]
=#

using QuanserComponents
import QuanserComponents as QC
using Statistics, Random, Printf
using Plots

const Ts = 0.01
const Tf = 10.0
const NROLL = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 1000
const OUTDIR = length(ARGS) >= 2 ? ARGS[2] : "mpc_rollouts"
const PLANT = length(ARGS) >= 3 ? Symbol(ARGS[3]) : :identified
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
    X = zeros(4, N); U = zeros(N); flag = zeros(N); texec = zeros(N)
    for i in 1:N
        t0 = time_ns()
        out = ctrl()
        texec[i] = (time_ns() - t0) / 1e9
        X[:, i] .= s.x; U[i] = applied[]; flag[i] = out.exitflag
        step!(s, applied[])
    end
    return (; X, U, flag, texec)
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
# The simulated plant: the prediction model itself, or the same model with other parameters.
plant_params = PLANT === :identified ? nothing :
               PLANT === :nominal ? QC.nominal :
               PLANT === :perturbed ? QC.withparams(QC.identified; kt = 0.85 * QC.identified.kt, mr = 1.2 * QC.identified.mr,
                                                    Jp = 1.15 * QC.identified.Jp, bp = 2 * QC.identified.bp) :
               error("plant must be identified, perturbed or nominal")
plant = plant_params === nothing ? dyn : QC.furuta_mpc_dynamics(; idparams = plant_params)
sim = FurutaSim(plant; Ts)

rng = Xoshiro(1)
sample_x0(rng) = [1.5 * (2rand(rng) - 1), 2pi * rand(rng), 3.0 * (2rand(rng) - 1), 10.0 * (2rand(rng) - 1)]
x0s = [sample_x0(rng) for _ in 1:NROLL]

tcatch = fill(NaN, NROLL); armmax = zeros(NROLL); umax = zeros(NROLL)
nbadflag = zeros(Int, NROLL); nmaxiter = zeros(Int, NROLL)
exec = Float64[]
NKEEP = min(NROLL, 100)                    # trajectories kept for the overlay plot
kept = Vector{Any}(undef, NKEEP)
twall = @elapsed for (i, x0) in enumerate(x0s)
    r = rollout!(ctrl, sim, x0)
    tcatch[i] = catch_time(r.X)
    armmax[i] = maximum(abs, r.X[1, :])
    umax[i] = maximum(abs, r.U)
    nbadflag[i] = count(!=(0), r.flag); nmaxiter[i] = count(==(2), r.flag)
    append!(exec, r.texec)
    i <= NKEEP && (kept[i] = r)
    i % 50 == 0 && (@printf("%4d/%d rollouts, %d successes so far\n", i, NROLL, count(!isnan, tcatch[1:i])); flush(stdout))
end

success = .!isnan.(tcatch)
q(v, p) = isempty(v) ? NaN : quantile(v, p)
med(v) = isempty(v) ? NaN : median(v)
mx(v) = isempty(v) ? NaN : maximum(v)
summary = @sprintf("""
    MPC Monte Carlo: %d rollouts of %.0f s, Ts = %.3f s, Np = 60, plant = %s
    successes (upright within %.2f rad for the last %.1f s): %d / %d = %.1f%%
    catch time [s]: median %.2f, 90%% %.2f, max %.2f
    arm excursion max |phi| [rad]: median %.2f, max %.2f (end stops at %.2f); rollouts beyond the stops: %d
    |u| max over all rollouts: %.2f V
    acados exitflag != 0 on %d of %d ticks (status 2, iteration limit, on %d)
    tick execution time [ms]: median %.3f, 99%% %.3f, max %.3f
    wall time %.1f min (%.2f ms per tick incl. the plant simulation)
    """, NROLL, Tf, Ts, PLANT, CATCH_TOL, HOLD, count(success), NROLL, 100mean(success),
    med(tcatch[success]), q(tcatch[success], 0.9), mx(tcatch[success]),
    median(armmax), maximum(armmax), ARM_LIMIT, count(>(ARM_LIMIT), armmax), maximum(umax),
    sum(nbadflag), NROLL * round(Int, Tf / Ts), sum(nmaxiter),
    1e3med(exec), 1e3q(exec, 0.99), 1e3mx(exec),
    twall / 60, 1e3twall / (NROLL * Tf / Ts))
println(summary)
write(joinpath(OUTDIR, "summary.txt"), summary)
open(joinpath(OUTDIR, "rollouts.csv"), "w") do io
    println(io, "shoulder0\telbow0\tshoulder_vel0\telbow_vel0\tcatch_time\tarm_max\tu_max\tbad_exitflags\tmaxiter_ticks")
    for i in 1:NROLL
        println(io, join(string.([x0s[i]; tcatch[i]; armmax[i]; umax[i]; nbadflag[i]; nmaxiter[i]]), '\t'))
    end
end
# Raw tick times, for re-plotting without re-running.
open(joinpath(OUTDIR, "exec_times.tsv"), "w") do io
    println(io, "exec_ms")
    for v in exec; println(io, 1e3v); end
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

# 4. Per-tick execution time. Ticks beyond the axis are counted in the title.
tmax = 1e3Ts
nover = count(>(tmax), 1e3exec)
p4 = histogram(1e3exec, bins = 0:(tmax / 100):tmax, color = BLUE, linecolor = :white, label = "",
               xlabel = "controller execution time per tick [ms]", ylabel = "ticks",
               title = @sprintf("Tick execution time; %d of %d ticks beyond the %.0f ms clock period", nover, length(exec), tmax))
savefig(p4, joinpath(OUTDIR, "exec_times.png"))

# 5. Arm excursion against the end stops, the MPC's constraint.
bmax = max(2.2, ceil(maximum(armmax); digits = 1))
p5 = histogram(armmax, bins = 0:0.05:bmax, color = BLUE, linecolor = :white, label = "",
               xlabel = "largest |arm angle| during the rollout [rad]", ylabel = "rollouts",
               title = "Arm excursion versus the end stops")
vline!(p5, [ARM_LIMIT], color = ORANGE, ls = :dash, lw = 2, label = "end stops")
savefig(p5, joinpath(OUTDIR, "arm_excursion.png"))
println("plots written to ", abspath(OUTDIR))
