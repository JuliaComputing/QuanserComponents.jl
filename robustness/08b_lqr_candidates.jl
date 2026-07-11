# Discriminate the LQR redesign candidates empirically:
# - swingup success with 2 and 3 samples of actuation delay
# - LQR-only catch capability at the top: largest |ω| caught at α = π,
#   nominal and with +20% pendulum mass
# The implemented (old) gains are included for reference.

include("common.jl")
using Printf

candidates = [
    ("old",   [-9.625743176817387, 394.43972658274106, -7.461418005226849, 84.42279971138271]),
    ("Q2=10",  [-9.307, 119.591, -4.032, 13.836]),
    ("Q2=30",  [-5.409, 95.615, -2.686, 10.971]),
    ("Q2=100", [-2.979, 78.561, -1.765, 8.934]),
    ("Q2=300", [-1.726, 68.281, -1.232, 7.705]),
]

function lqr_overrides(sys, L)
    Pair[
        sys.swingup.lqrstabilizer.L1 => L[1],
        sys.swingup.lqrstabilizer.L2 => L[2],
        sys.swingup.lqrstabilizer.L3 => L[3],
        sys.swingup.lqrstabilizer.L4 => L[4],
        sys.qubependulum.shoulder_joint.render => false,
    ]
end

model, ssys = build_system()

"Largest |elbow velocity| caught at alpha = pi with the LQR always engaged."
function catch_capability(L; mp_scale = 1.0)
    mp = 0.024 * mp_scale
    wmax = 0.0
    for w in 0.5:0.5:8.0
        ov = vcat(lqr_overrides(ssys, L),
            perturbation_overrides(ssys; mp),
            Pair[
                ssys.swingup.catchcondition.use_ellipsoid => false
                ssys.swingup.catchcondition.th => 1e6
                ssys.qubependulum.elbow_joint.phi => pi
                ssys.qubependulum.elbow_joint.w => w
                ssys.qubependulum.shoulder_joint.phi => 0.0
            ])
        m = metrics(simulate(ssys, ov; tf = 3.0), ssys; window = 0.5, th = 1e6)
        m.success ? (wmax = w) : break
    end
    wmax
end

results = Dict()
for (name, L) in candidates
    c1 = catch_capability(L)
    c2 = catch_capability(L; mp_scale = 1.2)
    results[name] = (; c1, c2)
    @printf("%-7s catch capability at top: nominal %.1f rad/s, mp+20%% %.1f rad/s\n", name, c1, c2)
end

for n in (2, 3)
    model_d, ssys_d = build_system(delayed = true, delay_n = n)
    for (name, L) in candidates
        m = metrics(simulate(ssys_d, vcat(lqr_overrides(ssys_d, L), swingup_ics(ssys_d)); tf = 15.0), ssys_d)
        @printf("%-7s delay %d: success %s (catch %.2f)\n", name, n, m.success, m.catch_time)
    end
end
