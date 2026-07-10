# Design of the velocity-aware (ellipsoidal) catch condition.
#
# The switch to the LQR should engage only where the LQR can actually catch.
# We use the sublevel sets of the LQR value function V(x) = x'Sx (S from the
# discrete Riccati equation of the same design as LQRstabilizer) expressed in
# measured coordinates e = [θ, φ-π, θ̇, φ̇]. The threshold c_in is calibrated
# against the empirical catch-region grids of 04_catch_region.jl across the
# perturbed plants, then hysteresis keeps the LQR engaged until V > c_out.
#
# Outputs: the entries of S in measured coordinates and recommended
# thresholds, to be baked into the CatchCondition component defaults.

include("common.jl")
using ControlSystemsMTK, ControlSystemsBase
using LinearAlgebra

model, ssys = build_system()

op = Dict(
    ssys.qubependulum.elbow_joint.phi => π,
    ssys.qubependulum.shoulder_joint.phi => 0.0,
    ssys.qubependulum.elbow_joint.w => 0.0,
    ssys.qubependulum.shoulder_joint.w => 0.0,
    ssys.qubependulum.voltage => 0.0,
    ssys.elbow_sampler.u => 0.0,
    ssys.shoulder_sampler.u => 0.0,
)
outputs = [
    ssys.qubependulum.shoulder_angle
    ssys.qubependulum.elbow_angle
    ssys.qubependulum.shoulder_joint.w
    ssys.qubependulum.elbow_joint.w
]
P = named_ss(model, [ssys.u_plant], outputs;
    op,
    loop_openings = [ssys.u_plant, ssys.shoulder_y, ssys.elbow_y],
    warn_empty_op = true,
    additional_passes = [SynchToolkit.compile_lustre],
    MultibodyComponents.linsys...,
)
Pd = c2d(ss(P), Ts)

# Same weights as the LQR design in test/runtests.jl
Q1 = P.C' * Diagonal([1000.0, 10.0, 1.0, 1.0]) * P.C
Q2 = 10.0 * I(1)

S, _, L_dare = ControlSystemsBase.MatrixEquations.ared(Pd.A, Pd.B, Q2, Q1)
L_fresh = vec(L_dare * pinv(P.C))
L_baked = [-9.625743176817387, 394.43972658274106, -7.461418005226849, 84.42279971138271]
println("L fresh LQR design: ", round.(L_fresh, digits = 4))
println("L in component:     ", round.(L_baked, digits = 4))
if !isapprox(L_fresh, L_baked, rtol = 1e-2)
    println("NOTE: the implemented gains do not correspond to this plant/weight",
        " combination; using a Lyapunov function of the implemented loop instead.")
end

# Value function of the IMPLEMENTED closed loop: L acts on the measured
# outputs, so the state-space gain is L_baked' * C. Solve the discrete
# Lyapunov equation for the closed loop with the LQR-style cost as forcing.
Lx = reshape(L_baked, 1, 4) * P.C
Acl = Pd.A - Pd.B * Lx
@assert maximum(abs, eigvals(Acl)) < 1 "implemented gains do not stabilize the linearization"
Qbar = Matrix(Symmetric(Q1 + Lx' * Q2 * Lx))
S_impl = ControlSystemsBase.MatrixEquations.lyapd(Matrix(Acl'), Qbar)

# V in measured coordinates e = [θ, φ-π, θ̇, φ̇]: x = pinv(C)*e
Ci = pinv(P.C)
Sy = Symmetric(Ci' * S_impl * Ci)
println("\nS in measured coordinates:")
display(round.(Matrix(Sy), sigdigits = 6))

# Calibrate c_in against the empirical catch grids (slices e = [0, a, 0, w]).
using CSV, DataFrames
alpha_grid = range(-1.2, 1.2, length = 25)
omega_grid = range(-8.0, 8.0, length = 25)
V(a, w) = [0.0, a, 0.0, w]' * Sy * [0.0, a, 0.0, w]

plants = ["nominal", "mp_plus20", "coulomb", "quantized"]
Vfail = Inf  # smallest V among failed grid points, over all plants
for name in plants
    f = resultpath("catch_region_$name.csv")
    isfile(f) || (println("missing $f – run 04_catch_region.jl first"); continue)
    S_grid = Matrix(CSV.read(f, DataFrame))
    for (i, a) in enumerate(alpha_grid), (j, w) in enumerate(omega_grid)
        if !S_grid[i, j]
            global Vfail = min(Vfail, V(a, w))
        end
    end
end
c_in = 0.8 * Vfail  # safety margin below the first observed failure
c_out = 4 * c_in
println("\nsmallest failing V across plants: ", round(Vfail, sigdigits = 5))
println("recommended c_in  = ", round(c_in, sigdigits = 5))
println("recommended c_out = ", round(c_out, sigdigits = 5))

# Normalize so the engage threshold is 1 and emit the component defaults
Sn = Matrix(Sy) ./ c_in
rows = NamedTuple[]
for i in 1:4, j in i:4
    push!(rows, (; entry = "S$(i)$(j)", value = Sn[i, j]))
end
push!(rows, (; entry = "c_release", value = c_out / c_in))
CSV.write(resultpath("catch_condition_design.csv"), DataFrame(rows))
println("\nnormalized ellipsoid entries (engage at V = 1):")
for r in rows
    @printf("  parameter %s::Real = %.6g\n", r.entry, r.value)
end

# Coverage: how much of the empirically catchable set the ellipsoid retains
for name in plants
    f = resultpath("catch_region_$name.csv")
    isfile(f) || continue
    S_grid = Matrix(CSV.read(f, DataFrame))
    caught = [S_grid[i, j] for (i, a) in enumerate(alpha_grid), (j, w) in enumerate(omega_grid)]
    inell = [V(a, w) <= c_in for (i, a) in enumerate(alpha_grid), (j, w) in enumerate(omega_grid)]
    retained = count(inell .& caught) / max(count(caught), 1)
    false_pos = count(inell .& .!caught)
    println(rpad(name, 12), " retained ", round(100retained, digits = 1),
        "% of catchable set, false positives inside ellipsoid: ", false_pos)
end
