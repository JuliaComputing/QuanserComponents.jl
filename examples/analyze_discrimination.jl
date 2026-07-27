# =============================================================================
# Decide which parameter set describes the hardware, using data from the
# discriminating experiment (test/hardware_discrimination.jl).
#
# Builds a UKF for each candidate parameter set, runs both over the recorded
# input/output, and reports the log-likelihood ratio. A large positive ratio
# means the identified set explains the data better than the nominal one.
#
# It can also validate the experiment BEFORE any hardware is touched: with
# SYNTHETIC = true it generates noisy data from the identified model under the
# designed input and checks that (a) the likelihood ratio points the right way
# and (b) refitting from the nominal starting point recovers the identified
# parameters. That is the difference between an input that merely produces
# *different* predictions and one that is actually *informative*.
#
# ENVIRONMENT:  julia --project=examples examples/analyze_discrimination.jl
# =============================================================================

using QuanserComponents
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using MultibodyComponents
using SynchToolkit
using LinearAlgebra
using Statistics
using Printf
using Random
using DelimitedFiles
using StaticArrays
using Distributions
using LowLevelParticleFilters
import LowLevelParticleFilters as LLPF
import SeeToDee
using LeastSquaresOptim

# --- configuration -----------------------------------------------------------
SYNTHETIC = false
REFIT     = get(ENV, "REFIT", "true") == "true"
DATAFILE  = joinpath(pkgdir(QuanserComponents), "discrimination_experiment.csv")
TRAJFILE  = joinpath(pkgdir(QuanserComponents), "input_design.csv")
Ts_NOM    = 0.005
const SIGMA_ENC = 2pi/2048

# =============================================================================
## 1. Model, control function, parameter sets  (same recipe as input_design.jl)
# =============================================================================
@named world        = MultibodyComponents.World(render = false)
@named qubependulum = QuanserComponents.QubePendulum()
@named idmodel      = ModelingToolkit.System(Equation[], t; systems = [world, qubependulum])

inputs = [qubependulum.voltage]
sysio  = multibody(idmodel; inputs, additional_passes = [SynchToolkit.compile_lustre])
(f_oop, _), x_sym, ps, iosys =
    ModelingToolkit.generate_control_function(sysio, inputs; simplify = false, split = false)
nx, nu, ny = length(x_sym), 1, 2
@assert nx == 4

prob0 = ModelingToolkit.ODEProblem(iosys, Dict(qubependulum.voltage => 0.0), (0.0, Ts_NOM))
P0 = prob0.p
IDP_FIELDS = fieldnames(QuanserComponents.IdParams)
set_idp! = ModelingToolkit.setp(iosys, [getproperty(qubependulum, f) for f in IDP_FIELDS])

function params_for(idp)
    pc = copy(P0)
    set_idp!(pc, [getfield(idp, f) for f in IDP_FIELDS])
    pc
end

# =============================================================================
## 2. Data: either the real recording or a synthetic dry run
# =============================================================================
if SYNTHETIC
    Dtraj = readdlm(TRAJFILE)
    uvec  = Float64.(Dtraj[2:end, 2])
    Ts    = Ts_NOM
    ddyn0 = SeeToDee.Rk4(f_oop, Ts)
    Random.seed!(1)
    x = SVector{4,Float64}(0, 0, 0, 0)
    ymat = Matrix{Float64}(undef, length(uvec), 2)
    p_true = params_for(QuanserComponents.identified)   # pretend the rig is "identified"
    for k in eachindex(uvec)
        ymat[k, 1] = x[1] + SIGMA_ENC*randn()
        ymat[k, 2] = x[2] + SIGMA_ENC*randn()
        x = SVector{4,Float64}(ddyn0(x, SA[uvec[k]], p_true, (k-1)*Ts))
    end
    @info "synthetic dry run: data generated from the IDENTIFIED model" n=length(uvec)
else
    D    = readdlm(DATAFILE)
    D    = D[2:end, :]
    tvec = Float64.(D[:, 1])
    ymat = Float64.(D[:, 2:3])
    uvec = Float64.(D[:, 4])
    Ts   = median(diff(tvec))
    @info "loaded hardware data" DATAFILE n=length(uvec) Ts
end

yvv = SVector{2,Float64}.(eachrow(ymat))
uvv = SVector{1,Float64}.(uvec)

# =============================================================================
## 3. Filters for the two candidate parameter sets
# =============================================================================
discrete_dynamics = SeeToDee.Rk4(f_oop, Ts)
measurement(x, u, p, tt) = SA[x[1], x[2]]

R2 = Matrix(SIGMA_ENC^2 * I(ny))
# Process noise as an unmodelled torque disturbance per joint, correlating each
# joint's angle and velocity through the double integrator (see the R1 note in
# examples/pendulum_identification.jl). State order [arm, pend, pend_vel, arm_vel].
R1 = zeros(nx, nx)
R1[[1, 4], [1, 4]] .= LLPF.double_integrator_covariance_smooth(Ts, 1e0)
R1[[2, 3], [2, 3]] .= LLPF.double_integrator_covariance_smooth(Ts, 1e-2)
R1 += 1e-12 * I
x0 = SVector{nx,Float64}(yvv[1][1], yvv[1][2], 0, 0)
Pcov0 = Matrix(Diagonal([1e-4, 1e-4, 1e-1, 1e-1]))

make_ukf(p) = UnscentedKalmanFilter(discrete_dynamics, measurement, R1, R2,
                                    MvNormal(Vector(x0), Pcov0); ny, nu, p)

function evaluate(tag, idp)
    p  = params_for(idp)
    ll = LLPF.loglik(make_ukf(p), uvv, yvv, p)
    s  = forward_trajectory(make_ukf(p), uvv, yvv, p)
    e  = [yvv[k] - SA[s.x[k][1], s.x[k][2]] for k in eachindex(yvv)]
    rms_ticks = sqrt(mean(x -> sum(abs2, x), e)) / SIGMA_ENC
    @printf("%-12s  loglik %14.1f   innovation rms %8.2f ticks\n", tag, ll, rms_ticks)
    ll
end

println("\n================ model comparison ================")
ll_nom = evaluate("nominal",    QuanserComponents.nominal)
ll_id  = evaluate("identified", QuanserComponents.identified)
Δll = ll_id - ll_nom
@printf("log-likelihood ratio (identified - nominal): %+.1f\n", Δll)
println(Δll > 0 ? "=> the IDENTIFIED parameters explain the data better" :
                  "=> the NOMINAL parameters explain the data better")
println("==================================================\n")

# =============================================================================
## 4. Optional refit from the nominal starting point
# =============================================================================
if REFIT
    tunable_syms = [qubependulum.Jp, qubependulum.br,
                    qubependulum.bp, qubependulum.kt, qubependulum.mr]
    set_tun! = ModelingToolkit.setp(iosys, tunable_syms)
    get_tun  = ModelingToolkit.getp(iosys, tunable_syms)

    p_start   = params_for(QuanserComponents.nominal)
    p_guess   = collect(get_tun(p_start))
    p_ref     = collect(get_tun(params_for(QuanserComponents.identified)))

    function dyn_tunable(x, u, ptun, tt)
        pc = copy(p_start)
        set_tun!(pc, ptun)
        f_oop(x, u, pc, tt)
    end
    ddyn_t = SeeToDee.Rk4(dyn_tunable, Ts)
    ukf_t(p) = UnscentedKalmanFilter(ddyn_t, measurement, R1, R2,
                                     MvNormal(Vector(x0), Pcov0); ny, nu, p)

    function residuals!(res, plog)
        p = exp10.(plog)
        ok = false
        try
            LLPF.prediction_errors!(res, ukf_t(p), uvv, yvv, p)
            ok = all(isfinite, res)
        catch
            ok = false
        end
        ok || fill!(res, 1e3)
        res
    end

    @info "refitting from the NOMINAL starting point"
    r = optimize!(LeastSquaresProblem(x = log10.(abs.(p_guess) .+ 1e-30), f! = residuals!,
                                      output_length = length(yvv)*ny, autodiff = :central),
                  LevenbergMarquardt(), show_trace = true, show_every = 5, iterations = 30)
    p_fit = exp10.(r.minimizer)

    println("\n================ refit from nominal ================")
    for (k, s) in enumerate(tunable_syms)
        short = split(string(s), "₊")[end]
        @printf("%-8s start %.4e   fitted %.4e   identified %.4e\n",
                short, p_guess[k], p_fit[k], p_ref[k])
    end
    println("====================================================\n")

    function print_idparams(p, syms)
        println("# paste into dyad/definitions.jl (updates the `identified` set):")
        println("const identified = withparams(nominal;")
        for (k, s) in enumerate(syms)
            field = split(string(s), "₊")[end]   # IdParams field == parameter short name
            @printf("    %-7s = %.8g,\n", field, p[k])
        end
        println(")\n")
    end
    print_idparams(p_fit, tunable_syms)

end
