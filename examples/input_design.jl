# =============================================================================
# Discriminating input design: nominal vs identified QubePendulum parameters.
#
# Designs a 60 s open-loop voltage trajectory that maximises the difference
# between the responses predicted by `QuanserComponents.nominal` and
# `QuanserComponents.identified`, so a single hardware run can tell which
# parameter set describes the real QUBE.
#
# Two stages:
#   1. Linearize both models about the hanging equilibrium and locate the
#      frequency band where |G_id(iw) - G_nom(iw)| is largest. The dominant
#      effect is the pendulum resonance: the two sets predict ~1.3 vs ~1.8 Hz.
#   2. Optimise the amplitudes of a periodic multisine on that band, maximising
#      the weighted output difference of the *nonlinear* models subject to a
#      barrier that keeps the arm well inside its end stops and stops the
#      pendulum from going over the top.
#
# The input is periodic with period T_PERIOD and repeated N_REPEAT times, which
# keeps the optimisation cheap (score one period, not sixty seconds) and gives
# periodic averaging when the data is analysed.
#
# ENVIRONMENT:  julia --project=examples examples/input_design.jl
# =============================================================================

using QuanserComponents
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using MultibodyComponents
using SynchToolkit
using ControlSystemsMTK
using ControlSystemsBase
using LinearAlgebra
using Statistics
using Printf
using DelimitedFiles
using StaticArrays
using Optim
import SeeToDee

# --- configuration -----------------------------------------------------------
const Ts        = 0.005
const T_PERIOD  = 10.0                     # one multisine period
const N_PERIOD  = round(Int, T_PERIOD/Ts)  # 2000 samples
const N_REPEAT  = 6                        # -> 60 s total
const UMAX      = 3.0                      # V, the empirically safe magnitude
const ARM_SOFT  = 1.9198621771937625       # 110 deg, the model's arm_limit
const ARM_DES   = deg2rad(70)              # design margin (40 deg of headroom)
const ELBOW_MAX = deg2rad(150)             # stay short of going over the top
const SIGMA_ENC = 2pi/2048                  # one encoder tick [rad]
const OUTFILE   = joinpath(pkgdir(QuanserComponents), "input_design.csv")

# =============================================================================
## 1. Model, control function, and the two parameter sets
# =============================================================================
@named world        = MultibodyComponents.World(render = false)
@named qubependulum = QuanserComponents.QubePendulum()
@named idmodel      = ModelingToolkit.System(Equation[], t; systems = [world, qubependulum])

inputs = [qubependulum.voltage]
sysio  = multibody(idmodel; inputs, additional_passes = [SynchToolkit.compile_lustre])
(f_oop, _), x_sym, ps, iosys =
    ModelingToolkit.generate_control_function(sysio, inputs; simplify = false, split = false)
nx = length(x_sym)
@assert nx == 4   # [shoulder_angle, elbow_angle, elbow_vel, shoulder_vel]

prob0 = ModelingToolkit.ODEProblem(iosys, Dict(qubependulum.voltage => 0.0), (0.0, Ts))
const P0 = prob0.p

const IDP_FIELDS = fieldnames(QuanserComponents.IdParams)
set_idp! = ModelingToolkit.setp(iosys, [getproperty(qubependulum, f) for f in IDP_FIELDS])

"MTKParameters realising a full `IdParams` set."
function params_for(idp)
    pc = copy(P0)
    set_idp!(pc, [getfield(idp, f) for f in IDP_FIELDS])
    pc
end
const P_NOM = params_for(QuanserComponents.nominal)
const P_ID  = params_for(QuanserComponents.identified)

const ddyn = SeeToDee.Rk4(f_oop, Ts)

"Simulate the plant from rest under voltage sequence `u`. Returns nx x N states."
function rollout(u::AbstractVector, p)
    x = SVector{4,Float64}(0, 0, 0, 0)
    X = fill(NaN, 4, length(u))        # NaN-filled so a bail-out is detectable
    try
        @inbounds for k in eachindex(u)
            X[:, k] = x
            x = SVector{4,Float64}(ddyn(x, SA[u[k]], p, (k-1)*Ts))
            all(isfinite, x) || return X   # diverged: the rest stays NaN
        end
    catch
        return X   # a wild swing can make the inertia solve singular
    end
    X
end

# =============================================================================
## 2. Stage 1 - where do the two models differ?
# =============================================================================
"Linearize about the hanging equilibrium for a given `IdParams` set."
function linearize_at(idp)
    op = Dict{Any,Any}(
        qubependulum.elbow_joint.phi    => 0.0,   # hanging down
        qubependulum.shoulder_joint.phi => 0.0,
        qubependulum.elbow_joint.w      => 0.0,
        qubependulum.shoulder_joint.w   => 0.0,
        qubependulum.voltage            => 0.0,
    )
    for f in IDP_FIELDS
        op[getproperty(qubependulum, f)] = getfield(idp, f)
    end
    outs = [qubependulum.shoulder_angle, qubependulum.elbow_angle]
    named_ss(idmodel, [qubependulum.voltage], outs; op, MultibodyComponents.linsys...)
end

P_lin_nom = linearize_at(QuanserComponents.nominal)
P_lin_id  = linearize_at(QuanserComponents.identified)

w      = exp10.(LinRange(-1, 2, 500))                 # rad/s
G_nom  = freqresp(ss(P_lin_nom), w)
G_id   = freqresp(ss(P_lin_id),  w)
# discrimination density: difference in predicted output per volt, in encoder ticks
Δmag   = [norm(G_id[:, 1, i] - G_nom[:, 1, i]) / SIGMA_ENC for i in eachindex(w)]
f_hz   = w ./ (2π)

let i = argmax(Δmag)
    @info "stage 1: frequency screening" peak_f_Hz=round(f_hz[i], digits=3) peak_Δ_ticks_per_V=round(Δmag[i], digits=1)
end

# =============================================================================
## 3. Stage 2 - periodic multisine, amplitudes optimised on the nonlinear models
# =============================================================================
# Harmonics of 1/T_PERIOD inside the informative band. No DC / near-DC content:
# the arm has no restoring force, so low frequencies are exactly what makes it
# walk into an end stop during an open-loop run.
const F_MIN = 0.4    # Hz
const F_MAX = 3.6    # Hz
const HARM  = [k for k in 1:round(Int, F_MAX*T_PERIOD) if k/T_PERIOD >= F_MIN]
const FREQS = HARM ./ T_PERIOD
const NF    = length(FREQS)

# Schroeder phases give a low crest factor, so more energy fits under |u| <= UMAX.
schroeder_phases(n) = [-π*k*(k-1)/n for k in 1:n]
const PHI0 = schroeder_phases(NF)

const TVEC_P = (0:N_PERIOD-1) .* Ts

"""
Build one period of the multisine. `c = [log-amplitudes; logit-scale]`; the
signal is peak-normalised then scaled by `s ∈ (0,1]`, so `|u| <= UMAX` holds by
construction and no input penalty is needed.
"""
function build_period(c)
    a = exp.(@view c[1:NF])
    s = 1 / (1 + exp(-c[NF+1]))           # (0,1)
    u = zeros(eltype(c), N_PERIOD)
    @inbounds for (j, f) in enumerate(FREQS)
        aj, φj = a[j], PHI0[j]
        for k in 1:N_PERIOD
            u[k] += aj * sin(2π*f*TVEC_P[k] + φj)
        end
    end
    m = maximum(abs, u)
    m > 0 ? (UMAX * s / m) .* u : u
end

"Mean squared constraint excess [rad^2] over a trajectory: 0 iff feasible."
function excess(X)
    b = 0.0
    @inbounds for k in axes(X, 2)
        e = abs(X[1, k]) - ARM_DES;   e > 0 && (b += e^2)
        g = abs(X[2, k]) - ELBOW_MAX; g > 0 && (b += g^2)
    end
    b / size(X, 2)
end

"Peak arm and elbow excursion [rad]."
peaks(X) = (maximum(abs, @view X[1, :]), maximum(abs, @view X[2, :]))

"""
Discrimination score for one period of input, in mean squared encoder ticks.
Two periods are simulated and only the second is scored, so the transient does
not dominate and the number reflects the periodic steady state that the repeated
60 s experiment will actually spend most of its time in.
"""
function score(u_period)
    u2 = repeat(u_period, 2)
    Xn = rollout(u2, P_NOM)
    Xi = rollout(u2, P_ID)
    # A diverged roll gets a large *finite* violation: Inf would break the line search.
    (all(isfinite, Xn) && all(isfinite, Xi)) || return (0.0, 1e3, Xn, Xi)
    idx = N_PERIOD+1:2N_PERIOD
    D = 0.0
    @inbounds for k in idx, r in 1:2
        D += ((Xi[r, k] - Xn[r, k]) / SIGMA_ENC)^2
    end
    (D / length(idx), excess(Xn) + excess(Xi), Xn, Xi)
end

# `D` spans several orders of magnitude (it explodes once the pendulum starts
# rotating and the two models desynchronise completely, which is *not* useful
# discrimination). `log1p` compresses it so a moderate penalty weight reliably
# dominates any constraint violation, making the feasible set the binding one.
function objective(c)
    D, viol, _, _ = score(build_period(c))
    -log1p(D) + 1e4 * viol
end

"Largest gain in (0,1] on `c`'s scale entry keeping BOTH models feasible."
function feasible_scale(c; lo = 0.0, hi = 1.0, iters = 24)
    isfeas(s) = (cc = copy(c); cc[NF+1] = log(s/(1-s)); score(build_period(cc))[2] == 0.0)
    isfeas(hi - 1e-6) && return hi - 1e-6
    for _ in 1:iters
        mid = (lo + hi)/2
        isfeas(mid) ? (lo = mid) : (hi = mid)
    end
    lo
end

# Warm start: amplitudes weighted by the stage-1 discrimination density, with the
# overall scale bisected down to the largest value that is feasible for BOTH
# models. The arm bound -- not the 3 V limit -- is what binds here: even a 0.5 V
# sine near the resonance already swings the arm to ~70 deg.
Δat(f) = Δmag[argmin(abs.(f_hz .- f))]
c0 = vcat(log.([max(Δat(f), 1e-6) for f in FREQS] ./ maximum(Δat.(FREQS))), 0.0)
let s = feasible_scale(c0)
    global c0[NF+1] = log(s/(1-s))
    @info "warm start scaled to feasibility" scale=round(s, digits=4) peak_V=round(UMAX*s, digits=3)
end

@info "stage 2: optimising" n_frequencies=NF band_Hz=(first(FREQS), last(FREQS))
let (D, viol, Xn, Xi) = score(build_period(c0))
    @printf("  warm start: rms diff %.1f ticks, violation %.3g, arm %.1f/%.1f deg\n",
            sqrt(D), viol, rad2deg(first(peaks(Xn))), rad2deg(first(peaks(Xi))))
end

res = Optim.optimize(
    objective, c0,
    LBFGS(),
    Optim.Options(iterations = 60, show_trace = true, show_every = 5, time_limit = 1800),
)
c_opt = Optim.minimizer(res)
# The penalty is soft, so enforce feasibility exactly before shipping the design.
let s = feasible_scale(c_opt)
    c_opt[NF+1] = log(s/(1-s))
    @info "final design scaled to feasibility" scale=round(s, digits=4) peak_V=round(UMAX*s, digits=3)
end

u_period = build_period(c_opt)
u_full   = repeat(u_period, N_REPEAT)
tvec     = (0:length(u_full)-1) .* Ts

# Raised-cosine fade-in over the first second: the periodic signal starts at a
# non-zero value, and slamming the motor with a step excites dynamics we have not
# modelled. Only the first period is affected; the rest stays exactly periodic.
let nfade = round(Int, 1.0/Ts)
    @inbounds for k in 1:nfade
        u_full[k] *= 0.5 * (1 - cos(π*(k-1)/nfade))
    end
end

# =============================================================================
## 4. Stage 3 - validation and comparison against a plain chirp
# =============================================================================
"Logarithmic chirp, ±1, as a baseline excitation (equivalent to CSI's `chirp`)."
function logchirp(N; Ts, f0, f1)
    T = N*Ts
    [sin(2π * (T*f0/log(f1/f0)) * ((f1/f0)^((k-1)*Ts/T) - 1)) for k in 1:N]
end

"Scale a fixed waveform down to the largest amplitude feasible for both models."
function scale_to_feasible(u_shape; hi = UMAX, iters = 24)
    isfeas(a) = score(a .* u_shape)[2] == 0.0
    isfeas(hi) && return hi .* u_shape
    lo = 0.0
    for _ in 1:iters
        mid = (lo + hi)/2
        isfeas(mid) ? (lo = mid) : (hi = mid)
    end
    lo .* u_shape
end

# Scaled to feasibility so the comparison is like-for-like: at full 3 V the chirp
# spins both the arm and the pendulum right through their limits.
u_chirp = scale_to_feasible(logchirp(N_PERIOD; Ts, f0 = F_MIN, f1 = F_MAX))

function report(tag, u_period)
    D, viol, Xn, Xi = score(u_period)
    arm = max(first(peaks(Xn)), first(peaks(Xi)))
    elb = max(last(peaks(Xn)),  last(peaks(Xi)))
    @printf("%-22s  peak %.2f V   rms diff %8.1f ticks   arm %5.1f deg   elbow %6.1f deg   %s\n",
            tag, maximum(abs, u_period), sqrt(D), rad2deg(arm), rad2deg(elb),
            viol == 0 ? "feasible" : "*** BOUND VIOLATED ***")
    (; D, arm, elb, viol)
end

println("\n================ predicted discrimination (per sample) ================")
r_chirp = report("log chirp baseline", u_chirp)
r_opt   = report("optimised multisine", u_period)
@printf("improvement over chirp: %.2fx in rms output difference\n",
        sqrt(r_opt.D) / max(sqrt(r_chirp.D), eps()))
@printf("encoder noise is 1 tick, so the two models are separated by ~%.0f sigma per sample\n",
        sqrt(r_opt.D))
println("=======================================================================\n")

# =============================================================================
## 5. Write the trajectory
# =============================================================================
open(OUTFILE, "w") do io
    println(io, "time\tu")
    for k in eachindex(u_full)
        @printf(io, "%.6f\t%.6f\n", tvec[k], u_full[k])
    end
end
@info "wrote trajectory" OUTFILE n_samples=length(u_full) duration_s=tvec[end]+Ts maxabs_u=maximum(abs, u_full)

# =============================================================================
## 6. Diagnostic plots
# =============================================================================
using Plots
gr()

band = (F_MIN .<= f_hz .<= F_MAX)
p1 = plot(f_hz, Δmag; xscale = :log10, yscale = :log10, xlabel = "frequency [Hz]",
          ylabel = "|G_id - G_nom| [ticks/V]", title = "Stage 1: where the models differ",
          lab = "discrimination density", lw = 2)
vline!(p1, [F_MIN, F_MAX]; ls = :dash, c = :black, lab = "excitation band")
scatter!(p1, FREQS, Δat.(FREQS); ms = 3, lab = "harmonics used")

_, _, Xn, Xi = score(u_period)
tp = (0:2N_PERIOD-1) .* Ts
p2 = plot(tvec[1:N_PERIOD], u_period; xlabel = "time [s]", ylabel = "u [V]",
          title = "Designed input (one 10 s period)", lab = "", lw = 1)
p3 = plot(tp, rad2deg.(Xn[1, :]); xlabel = "time [s]", ylabel = "arm [deg]",
          lab = "nominal", title = "Predicted arm response", lw = 1)
plot!(p3, tp, rad2deg.(Xi[1, :]); lab = "identified", lw = 1)
hline!(p3, [-rad2deg(ARM_DES), rad2deg(ARM_DES)]; ls = :dash, c = :black, lab = "design bound")
p4 = plot(tp, rad2deg.(Xn[2, :]); xlabel = "time [s]", ylabel = "pendulum [deg]",
          lab = "nominal", title = "Predicted pendulum response", lw = 1)
plot!(p4, tp, rad2deg.(Xi[2, :]); lab = "identified", lw = 1)

plt = plot(p1, p2, p3, p4; layout = (2, 2), size = (1200, 800), legend = :topright)
figfile = joinpath(pkgdir(QuanserComponents), "input_design.png")
savefig(plt, figfile)
@info "wrote figure" figfile
