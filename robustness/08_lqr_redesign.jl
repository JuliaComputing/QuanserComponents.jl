# LQR redesign from the actual model linearization.
#
# The gains baked into LQRstabilizer do not correspond to an LQR design from
# this model's linearization with the weights documented in test/runtests.jl
# (they are far more aggressive), and the campaign showed the resulting loop
# has essentially no delay margin. This script designs fresh gains, sweeping
# the control weight to buy input margins, and validates each candidate in
# closed-loop simulation before selecting.
#
# Selection criteria:
# - delay margin of the state-feedback loop at least two samples
# - nominal swingup (robust defaults) still catches promptly
# - catches with one sample of actuation delay
# - catches a hard perturbed plant from the MC campaign

include("common.jl")
using ControlSystemsMTK, ControlSystemsBase
using LinearAlgebra
using Printf

model, ssys = build_system()
model_d, ssys_d = build_system(delayed = true, delay_n = 1)

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
Ci = pinv(P.C)

"Loop-transfer margins of discrete state feedback u = -L*x at the plant input."
function loop_margins(L_y)
    Lx = reshape(L_y, 1, 4) * P.C
    Lt = ss(Pd.A, Pd.B, Lx, 0, Ts)  # loop transfer L(z)
    wgm, gm, wpm, pm = margin(Lt)
    delay_margin_s = deg2rad(only(pm)) / only(wpm)
    (; gm = only(gm), pm = only(pm), delay_samples = delay_margin_s / Ts)
end

L_baked = [-9.625743176817387, 394.43972658274106, -7.461418005226849, 84.42279971138271]
m0 = loop_margins(L_baked)
@printf("implemented gains: gm %.2f, pm %.1f deg, delay margin %.2f samples\n",
    m0.gm, m0.pm, m0.delay_samples)

designs = Dict{Float64, Vector{Float64}}()
for q2 in (10.0, 30.0, 100.0, 300.0)
    Q1 = P.C' * Diagonal([1000.0, 10.0, 1.0, 1.0]) * P.C
    Q1 = Matrix(Symmetric(Q1))
    _, _, F = ControlSystemsBase.MatrixEquations.ared(Pd.A, Pd.B, q2 * I(1), Q1)
    L = vec(F * Ci)
    designs[q2] = L
    m = loop_margins(L)
    @printf("Q2 = %6.1f: L = [%8.3f %8.3f %8.3f %8.3f]  gm %.2f  pm %.1f deg  delay %.2f samples\n",
        q2, L..., m.gm, m.pm, m.delay_samples)
end

# Simulation validation of each candidate (robust swingup defaults).
# Hard plant: the mildest failing MC draw of the original campaign.
hard = (; mp = 0.0244341173211006, Lp = 0.12963014151022853,
    mr = 0.08900340167288685, kt = 0.044360644011252365,
    km = 0.03913861399268383, Rm = 7.97187181423963,
    br = 0.0005855064113230941, bp = 4.786994165706217e-5,
    tau_c_sh = 0.0040382876985633346, tau_c_el = 0.0003770799004443717,
    quantized = true)

function lqr_overrides(sys, L)
    Pair[
        sys.swingup.lqrstabilizer.L1 => L[1],
        sys.swingup.lqrstabilizer.L2 => L[2],
        sys.swingup.lqrstabilizer.L3 => L[3],
        sys.swingup.lqrstabilizer.L4 => L[4],
        sys.qubependulum.shoulder_joint.render => false,
    ]
end

println()
for q2 in sort(collect(keys(designs)))
    L = designs[q2]
    m_nom = metrics(simulate(ssys, vcat(lqr_overrides(ssys, L), swingup_ics(ssys)); tf = 10.0), ssys)
    m_del = metrics(simulate(ssys_d, vcat(lqr_overrides(ssys_d, L), swingup_ics(ssys_d)); tf = 15.0), ssys_d)
    m_hard = metrics(simulate(ssys,
        vcat(lqr_overrides(ssys, L), perturbation_overrides(ssys; hard...), swingup_ics(ssys)); tf = 20.0), ssys)
    @printf("Q2 = %6.1f: nominal %s (catch %.2f)  delay1 %s (catch %.2f)  hard %s (catch %.2f)\n",
        q2, m_nom.success, m_nom.catch_time, m_del.success, m_del.catch_time,
        m_hard.success, m_hard.catch_time)
end
