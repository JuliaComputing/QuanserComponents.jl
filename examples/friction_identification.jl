# =============================================================================
# Friction identification for the QuanserComponents Furuta pendulum.
#
# Fits the `Friction` component's `FrictionParams` to a log written by the
# `FurutaFrictionExperiment` analysis, which drives the arm around a staircase of
# constant speeds (see dyad/friction.dyad).
#
# The idea is the one from QuanserInterface/examples/estimate_friction.jl: at a
# genuinely constant velocity the equation of motion
#
#     τ_motor = J ω̇ + τ_f(ω)
#
# loses its inertia term, so the applied torque *is* the friction torque and no
# inertia has to be modelled. Everything below therefore hinges on selecting only
# the samples where the acceleration really is negligible; that selection is the
# part worth looking at the plots for.
#
# The fit itself is linear least squares on
#
#     u(ω) = a₁ sign(ω) + a₂ ω + a₃ sign(ω) ω² + a₄ ω³
#
# in *motor-command* space, because that is what is measured. Converting to the
# torque space that `FrictionParams` uses needs the motor constants, and folding
# out the back-EMF term:
#
#     τ_f(ω) = kt/Rm ⋅ (u − km ω)
#
# so a₂ loses km before scaling. That is exactly the quantity `QubePendulum`'s
# damper calls `br + kt km / Rm`, which is why the printed `kv` is directly
# comparable with `IdParams.br`.
#
# ENVIRONMENT: run in the package `examples/` environment, which has Plots:
#   jld --project=examples run examples/friction_identification.jl
# =============================================================================

using QuanserComponents
using QuanserComponents: read_friction_log, FrictionParams, friction_nominal,
                         FRICTION_LOG_FILE, identified, withparams
using Statistics
using Printf
using LinearAlgebra
using Plots

# --- configuration ----------------------------------------------------------
LOG_PATH = joinpath(pkgdir(QuanserComponents), FRICTION_LOG_FILE)

# Data selection. Tune these by looking at the diagnostic plots below.
SETTLE     = 0.6     # [s] discarded after each change of the velocity reference
ACC_TOL    = 3.0     # [rad/s²] max |arm acceleration| to count as constant speed
ELBOW_TOL  = 1.5     # [rad/s] max |pendulum velocity|; the pendulum must be quiet too
W_MIN_FIT  = 0.5     # [rad/s] exclude near-standstill, where the sign term is undefined
W_MAX_FIT  = 40.0    # [rad/s] exclude anything faster than the controller is used at
SMOOTH_N   = 20      # samples in the moving average applied to the acceleration

# Motor constants used to turn the fitted command into a torque. The identified set,
# since that is what the plant these numbers feed back into uses.
MOTOR = identified

# =============================================================================
## 1. Load the log the program wrote
# =============================================================================
isfile(LOG_PATH) || error("""
    No log at $LOG_PATH. Collect one first:

        using QuanserComponents
        FurutaFrictionExperiment(; arm_deg = 0)

    (the analysis writes `$(FRICTION_LOG_FILE)` in the working directory by default).""")

D  = read_friction_log(LOG_PATH)
Ts = median(diff(D.time))
N  = length(D.time)
@info "loaded log" LOG_PATH nsamples=N Ts duration=D.time[end]

tvec = D.time
wref = D.w_ref
w    = D.shoulder_velocity            # already filtered, by the program's VelocityEstimator
u    = D.control_input
elb  = D.elbow_angle

# =============================================================================
## 2. Derived signals for the data selection
# =============================================================================
"Central difference, with the ends held so the result stays the same length."
function centraldiff(x)
    d = similar(x)
    @inbounds for i in 2:(length(x) - 1)
        d[i] = (x[i + 1] - x[i - 1]) / 2
    end
    d[1] = d[2]
    d[end] = d[end - 1]
    return d
end

"Zero-phase moving average of width `n` (forward then backward, as `filtfilt` does)."
function smooth(x, n)
    n <= 1 && return copy(x)
    ma(v) = [mean(@view v[max(1, i - n + 1):i]) for i in eachindex(v)]
    return reverse(ma(reverse(ma(x))))
end

acc  = smooth(centraldiff(w) ./ Ts, SMOOTH_N)      # arm angular acceleration
welb = centraldiff(elb) ./ Ts                      # pendulum angular velocity

# Time since the reference last changed, so the transient after each step can go.
since_step = similar(tvec)
let last_change = tvec[1], prev = wref[1]
    for i in eachindex(tvec)
        wref[i] == prev || (last_change = tvec[i]; prev = wref[i])
        since_step[i] = tvec[i] - last_change
    end
end

keep = (since_step .>= SETTLE) .&
       (abs.(acc) .< ACC_TOL) .&
       (abs.(welb) .< ELBOW_TOL) .&
       (abs.(w) .>= W_MIN_FIT) .& (abs.(w) .<= W_MAX_FIT)

nkeep = count(keep)
@info "data selection" nkeep frac=round(nkeep / N, digits = 3) levels=length(unique(wref[keep]))
nkeep > 20 || error("""
    Only $nkeep samples survived the selection. Loosen SETTLE / ACC_TOL / ELBOW_TOL, or
    lengthen `t_step` in the experiment so each speed has time to settle.""")
# Both signs are what separates the Coulomb term from a constant offset.
(any(w[keep] .> 0) && any(w[keep] .< 0)) ||
    @warn "only one sign of velocity survived the selection; kc will be confounded with an offset"

# =============================================================================
## 3. Least squares in motor-command space
# =============================================================================
signsquare(x) = sign(x) * x^2

w1, u1 = w[keep], u[keep]
sc = maximum(abs, w1)                # scale for conditioning; sign() is scale-free
ws = w1 ./ sc
A  = [sign.(w1) ws signsquare.(ws) ws .^ 3]
a  = A \ u1
a[2] /= sc
a[3] /= sc^2
a[4] /= sc^3

resid = u1 .- [sign.(w1) w1 signsquare.(w1) w1 .^ 3] * a
@printf("\nfit over %d samples: residual RMS %.4f V (command range %.2f V)\n",
        nkeep, sqrt(mean(abs2, resid)), maximum(u1) - minimum(u1))

# =============================================================================
## 4. Convert to torque space
# =============================================================================
kt, Rm, km = MOTOR.kt, MOTOR.Rm, MOTOR.km
g = kt / Rm                                  # command [V] -> torque [N·m]
p_id = FrictionParams(; kc = g * a[1],
                        kv = g * (a[2] - km),   # back-EMF folded out
                        k2 = g * a[3],
                        k3 = g * a[4],
                        w_tanh = friction_nominal.w_tanh)   # not identifiable here

println("\n================ identified friction ================")
@printf("breakaway command   a1 = %+.4f V\n", a[1])
@printf("Coulomb             kc = %+.4e N·m\n", p_id.kc)
@printf("viscous             kv = %+.4e N·m·s/rad   (IdParams.br = %.4e)\n",
        p_id.kv, MOTOR.br)
@printf("quadratic           k2 = %+.4e\n", p_id.k2)
@printf("cubic               k3 = %+.4e\n", p_id.k3)
println("=====================================================")
if p_id.kc <= 0
    @warn """
        kc came out non-positive, which is not physical. The most likely cause is deadband
        compensation having been left on during the experiment: it adds a fixed offset in
        the direction of the command, straight onto the term being measured. Re-run with
        `card_options = "deadband_compensation=0.0"` (the analysis default)."""
end

# =============================================================================
## 5. Ready-to-paste `friction_identified` set for dyad/definitions.jl
# =============================================================================
function print_friction_params(p)
    println("\n# paste into dyad/definitions.jl (replaces `friction_identified`):")
    println("const friction_identified = withparams(friction_nominal;")
    for f in (:kc, :kv, :k2, :k3)
        @printf("    %-7s = %.8g,\n", f, getfield(p, f))
    end
    println(")")
    println("# then use it with e.g. Friction(params = friction_identified)\n")
end
print_friction_params(p_id)

# =============================================================================
## 6. Plots
# =============================================================================
# What the experiment did, and which samples the fit was allowed to see.
p1 = plot(tvec, [wref w], lab = ["reference" "measured"], ylabel = "ω [rad/s]",
          framestyle = :zerolines, title = "friction experiment")
scatter!(p1, tvec[keep], w[keep], lab = "selected", ms = 1.2, mc = :green, ma = 0.4)
p2 = plot(tvec, u, lab = "command", ylabel = "u [V]", framestyle = :zerolines)
p3 = plot(tvec, [acc welb], lab = ["arm acceleration" "pendulum velocity"],
          framestyle = :zerolines, xlabel = "t [s]")
hline!(p3, [-ACC_TOL ACC_TOL], lab = "", l = (:black, :dash))
plt_trace = plot(p1, p2, p3, layout = (3, 1), size = (900, 800))

# The fit itself: command against velocity, which is where a bad selection shows up
# as a fan of points rather than a curve.
wgrid = range(-sc, sc, length = 601)
uhat_hard = [sign.(wgrid) wgrid signsquare.(wgrid) wgrid .^ 3] * a
sw = tanh.(wgrid ./ p_id.w_tanh)
uhat_smooth = a[1] .* sw .+ a[2] .* wgrid .+ a[3] .* sw .* wgrid .^ 2 .+ a[4] .* wgrid .^ 3
plt_fit = scatter(w1, u1, lab = "selected data", ms = 2, ma = 0.5,
                  xlabel = "ω [rad/s]", ylabel = "u [V]", framestyle = :zerolines,
                  title = "friction model")
plot!(plt_fit, wgrid, uhat_hard, lab = "fit (sign)", lw = 2)
plot!(plt_fit, wgrid, uhat_smooth, lab = "fit (tanh, w_tanh = $(p_id.w_tanh))", lw = 2,
      l = :dash)

plot(plt_trace, plt_fit, layout = (1, 2), size = (1500, 800))
