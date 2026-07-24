# =============================================================================
# Parameter identification of the QuanserComponents Furuta pendulum.
#
# Estimates the two inertias (Jr, Jp) of the QubePendulum plant from a recorded
# swing-up trajectory, using a prediction-error method built on an Unscented
# Kalman Filter from LowLevelParticleFilters. The system is unstable and the
# recording contains a swing-up, so a state estimator is used to run the
# prediction-error identification robustly (same approach as
# QuanserInterface/examples/pendulum_identification.jl).
#
# The script is written so that fitting additional parameters later only
# requires editing `tunable_syms` below -- everything else is derived from it.
#
# ENVIRONMENT: run in the package `test/` environment, which is the only env
# that resolves the QuanserComponents multibody + codegen stack. E.g.
#   jld --project=<pkg>/test run examples/pendulum_identification.jl
# The extra identification deps (LowLevelParticleFilters, SeeToDee,
# LeastSquaresOptim, ForwardDiff, Distributions) were added to test/Project.toml.
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
using Distributions
using LowLevelParticleFilters
import LowLevelParticleFilters as LLPF
import SeeToDee
using LeastSquaresOptim

# --- configuration ----------------------------------------------------------
const DATA_PATH = joinpath(pkgdir(QuanserComponents), "swingup.csv")
const NDATA     = 1000         # number of samples to use

# QuanserInterface ML-optimized reference values (for comparison only)
const QI_OPTIMIZED = Dict("Jr" => 1.112300869775737e-5, "Jp" => 1.9808351489259391e-4)
# Gains currently baked into the LQRstabilizer component
const L_BAKED = [-2.8515070942708687, -24.415803244034326, -0.9920297324372649, -1.9975963404759338]

# =============================================================================
## 1. Load data
# =============================================================================
# Columns: time, shoulder_angle, elbow_angle, control_input  (tab-delimited).
D    = readdlm(DATA_PATH)                     # whitespace/tab split
D    = D[2:min(NDATA, size(D, 1)), :]         # drop header row
tvec = Float64.(D[:, 1])
ymat = Float64.(D[:, 2:3])                    # [shoulder_angle elbow_angle]
uvec = Float64.(D[:, 4])                      # control voltage

Ts  = median(diff(tvec))                      # absorbs the irregular first sample
yvv = SVector{2,Float64}.(eachrow(ymat))
uvv = SVector{1,Float64}.(uvec)
@info "loaded data" DATA_PATH nsamples=length(yvv) Ts

# =============================================================================
## 2. Build the identification model and nonlinear control function
# =============================================================================
# World supplies the gravity field (compiling QubePendulum alone drops gravity).
@named world        = MultibodyComponents.World(render = false)
@named qubependulum = QuanserComponents.QubePendulum()
@named idmodel      = ModelingToolkit.System(Equation[], t; systems = [world, qubependulum])

# Compile the multibody model as a control system with `voltage` designated as
# input. This yields the correct minimal 4-state realization
#   x = [shoulder_angle, elbow_angle, elbow_vel, shoulder_vel]
# (compiling before generate_control_function is essential: calling
#  generate_control_function directly on the raw model produces a pathological
#  6-state realization in which the input enters at the jerk level).
inputs = [qubependulum.voltage]
sysio = multibody(idmodel; inputs,
                  additional_passes = [SynchToolkit.compile_lustre])
(f_oop, _), x_sym, ps, iosys =
    ModelingToolkit.generate_control_function(sysio, inputs;
                                              simplify = false, split = false)

nx = length(x_sym)
nu = 1
ny = 2
@assert nx == 4

# Fully-constructed nominal parameter object (MTKParameters, incl. the DiffCache
# buffers created by the in-place inertia solve). Built via ODEProblem exactly
# as LowLevelParticleFiltersMTK does.
prob0 = ModelingToolkit.ODEProblem(iosys, Dict(qubependulum.voltage => 0.0), (0.0, Ts))
p0    = prob0.p

# =============================================================================
## 3. Tunable parameters  --  THE SINGLE EXTENSION POINT
# =============================================================================
tunable_syms = [
    qubependulum.Jr,
    qubependulum.Jp,
    qubependulum.br,
    qubependulum.bp,
]   # add .br, .bp, .kt, .Rm, ...
set_tunable! = ModelingToolkit.setp(iosys, tunable_syms)
get_tunable  = ModelingToolkit.getp(iosys, tunable_syms)
p_nominal    = collect(get_tunable(p0))

# =============================================================================
## 4. Dynamics wrapper (splices tunable params) + measurement + discretization
# =============================================================================
function continuous_dynamics(x, u, ptun, τ)
    pc = copy(p0)
    set_tunable!(pc, ptun)
    f_oop(x, u, pc, τ)
end
discrete_dynamics = SeeToDee.Rk4(continuous_dynamics, Ts)
measurement(x, u, p, τ) = SA[x[1], x[2]]

# =============================================================================
## 5. Filter setup
# =============================================================================
R2 = Matrix((2π / 2048) * I(ny))                       # measurement noise: one encoder tick
# Process noise models an unmodeled torque/acceleration disturbance on each joint.
# Such a disturbance enters through the double-integrator (angle ← velocity ←
# acceleration) structure, which correlates the angle and velocity noise of a joint
# — they are NOT independent, so R1 is block-structured rather than diagonal.
# `double_integrator_covariance_smooth(Ts, σ²) = σ²·[Ts³/3 Ts²/2; Ts²/2 Ts]` is the
# continuous-white-noise (full-rank, sample-time-invariant) discretization, which is
# the correct form here since the noise is additive to the RK4 output rather than
# integrated through it (see LowLevelParticleFilters "Discretization" docs).
# State order is [shoulder_angle, elbow_angle, elbow_vel, shoulder_vel], so each
# joint's (angle, velocity) pair sits at indices (1,4) for the shoulder and (2,3)
# for the elbow.
σ2_shoulder = 1e0      # shoulder acceleration-disturbance intensity (tune)
σ2_elbow    = 1e-2      # elbow    acceleration-disturbance intensity (tune)
R1 = zeros(nx, nx)
R1[[1, 4], [1, 4]] .= LLPF.double_integrator_covariance_smooth(Ts, σ2_shoulder)
R1[[2, 3], [2, 3]] .= LLPF.double_integrator_covariance_smooth(Ts, σ2_elbow)
R1 += 1e-12 * I        # tiny regularization for numerical positive-definiteness

# R1 = Matrix(Diagonal([1e-7, 1e-7, 1e-2, 1e-2])) 
x0 = SVector{nx,Float64}(yvv[1][1], yvv[1][2], 0, 0)
P0 = Matrix(Diagonal([1e-4, 1e-4, 1e-1, 1e-1]))        # initial state covariance

make_ukf(p) = UnscentedKalmanFilter(discrete_dynamics, measurement, R1, R2,
                                    MvNormal(Vector(x0), P0); ny, nu, p, names=SignalNames(name="UKF", x=string.(x_sym), y=["shoulde", "elbow"], u=string.(inputs)))

# =============================================================================
## 6. Sanity check with nominal params
# =============================================================================
sol0   = forward_trajectory(make_ukf(p_nominal), uvv, yvv)
innov0 = [yvv[k] - SA[sol0.x[k][1], sol0.x[k][2]] for k in eachindex(yvv)]
@info "nominal filter" loglik=LLPF.loglik(make_ukf(p_nominal), uvv, yvv, p_nominal) innov_rms=sqrt(mean(x->sum(abs2, x), innov0))
plot(sol0, plote=true, plotx=false, plotxt=false, plotxht=false, plotu=false)

# =============================================================================
## 7. Gauss-Newton estimation (Levenberg-Marquardt on whitened prediction errors)
# =============================================================================
# Optimize in log10-space for positivity and scaling. Non-finite residuals
# (filter divergence for a bad parameter guess) are replaced by a large penalty
# so Levenberg-Marquardt stays in the well-behaved region.
function residuals!(res, plog)
    p  = exp10.(plog)
    ok = false
    try
        LLPF.prediction_errors!(res, make_ukf(p), uvv, yvv, p)
        ok = all(isfinite, res)
    catch
        ok = false
    end
    ok || fill!(res, 1e3)
    return res
end

lsq    = LeastSquaresProblem(x = log10.(p_nominal), f! = residuals!,
                             output_length = length(yvv) * ny, autodiff = :central)
result = optimize!(lsq, LevenbergMarquardt(), show_trace=true, show_every=1, iterations=25)
p_id   = exp10.(result.minimizer)
sol_id   = forward_trajectory(make_ukf(p_id), uvv, yvv)
plot(sol_id, plote=true, plotx=false, plotxt=false, plotxht=false, plotu=false)

# =============================================================================
## 8. Report identified parameters
# =============================================================================
println("\n================ identified inertias ================")
for (k, s) in enumerate(tunable_syms)
    short = split(string(s), "₊")[end]
    qi    = get(QI_OPTIMIZED, short, nothing)
    @printf("%-6s nominal = %.4e   identified = %.4e%s\n",
            short, p_nominal[k], p_id[k],
            qi === nothing ? "" : @sprintf("   QI-opt = %.4e", qi))
end
println("=====================================================\n")

# =============================================================================
## 9. LQR redesign + comparison
# =============================================================================
# Linearize the plant about the upright equilibrium for a given (Jr, Jp) and
# design the discrete LQR gain, in the same convention as the baked-in gain
# (error vector [shoulder_angle, elbow_angle, shoulder_velocity, elbow_velocity]).
function design_L(Jrv, Jpv; Ts_lqr = 0.005)
    op = Dict(
        qubependulum.elbow_joint.phi    => π,
        qubependulum.shoulder_joint.phi => 0.0,
        qubependulum.elbow_joint.w      => 0.0,
        qubependulum.shoulder_joint.w   => 0.0,
        qubependulum.voltage            => 0.0,
        qubependulum.Jr                 => Jrv,
        qubependulum.Jp                 => Jpv,
    )
    outs = [qubependulum.shoulder_angle, qubependulum.elbow_angle,
            qubependulum.shoulder_joint.w, qubependulum.elbow_joint.w]
    P  = named_ss(idmodel, [qubependulum.voltage], outs; op, MultibodyComponents.linsys...)
    Pd = c2d(ss(P), Ts_lqr)
    Q1 = P.C' * Diagonal([1000.0, 10.0, 1.0, 1.0]) * P.C
    Q2 = 100.0 * I(1)
    vec(lqr(Pd, Q1, Q2) * pinv(P.C))
end

try
    L_nom = design_L(p_nominal[1], p_nominal[2])
    L_id  = design_L(p_id[1],      p_id[2])
    println("================ LQR gain comparison ================")
    names = ["L1 (θ_sh)", "L2 (φ_el)", "L3 (θ̇_sh)", "L4 (φ̇_el)"]
    @printf("%-12s %14s %14s %14s\n", "", "baked-in", "nominal-param", "identified")
    for i in 1:4
        @printf("%-12s %14.4f %14.4f %14.4f\n", names[i], L_BAKED[i], L_nom[i], L_id[i])
    end
    println("=====================================================")
catch e
    @warn "LQR redesign step failed" exception=(e, catch_backtrace())
end
