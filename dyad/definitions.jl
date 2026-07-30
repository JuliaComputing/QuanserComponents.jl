# Hand-written definitions included by the generated Dyad module.
#
# `IdParams` bundles the physical parameters of `QubePendulum` into a single
# value so a whole parameter set can be selected with one line:
#
#     QubePendulum(idparams = identified)
#
# and inside the Dyad component each parameter is read off it, e.g.
#
#     structural parameter idparams::Native = nominal
#     parameter Jr::Dyad.Inertia = idparams.Jr

"""
    IdParams

One complete set of `QubePendulum` physical parameters. The keyword defaults are
the datasheet / thin-rod nominal values, so `IdParams()` is the nominal set and
`IdParams(; Jr = ..., ...)` overrides only the fields you fit.
"""
Base.@kwdef struct IdParams
    Rm::Float64     = 8.4                                   # motor armature resistance
    kt::Float64     = 0.042                                 # current-to-torque constant
    mr::Float64     = 0.095                                 # rotary arm mass (incl. encoder)
    r::Float64      = 0.085                                 # rotary arm length
    #r_cm_r::Float64 = 0.085 / 2                             # rotary arm CoM radius
    #Jr::Float64     = 0.095 * 0.085^2 / 3 - 0.095 * (0.085 / 2)^2   # arm body inertia
    mp::Float64     = 0.024                                 # pendulum mass
    Lp::Float64     = 0.129                                 # pendulum length
    l::Float64      = 0.129 / 2                             # elbow-to-CoM distance
    Jp::Float64     = 7 * 0.024 * 0.129^2 / 12 - 0.024 * (0.129 / 2)^2   # pendulum body inertia
    bp::Float64     = 0.05 * 5e-5                           # pendulum viscous damping
end

"Copy of `p` with the named fields replaced (fill these from a fit's `p_id`)."
withparams(p::IdParams; changes...) =
    IdParams(; (f => get(changes, f, getfield(p, f)) for f in fieldnames(IdParams))...)

# The datasheet / thin-rod nominal set.
const nominal = IdParams()

# The identified set: start from nominal, override the fitted parameters.
# Replace the values below with your latest fit (the `p_id` printed by
# examples/pendulum_identification.jl for the corresponding `tunable_syms`).
const identified = withparams(nominal;
    Jp      = 2.5444125e-05,
    bp      = 1.4305747e-05,
    kt      = 0.054468594,
    mr      = 0.02399848,
)

# ===========================================================================
## Friction
# ===========================================================================

"""
    FrictionParams

Coefficients of the shoulder-axis friction model implemented by the `Friction`
component, in **torque** space:

```math
τ_f(ω) = k_c \\, σ(ω) + k_v ω + k_2 \\, σ(ω) ω^2 + k_3 ω^3
```

where `σ` is `sign` or, with `w_tanh > 0`, the smooth `tanh(ω / w_tanh)`. Only the
sign-odd terms carry `σ`, so `τ_f(-ω) = -τ_f(ω)`: friction always opposes the
motion.

This set *is* the whole speed-dependent torque of `QubePendulum`'s shoulder axis:
friction and back-EMF together, with no separate viscous coefficient and no `km`
anywhere. The individual coefficients are not separately interpretable:
with `k_2` and `k_3` present, `k_v` is not "the viscous part", and it can perfectly
well come out negative while the sum still gives damping that grows with speed, which
is what the device does. Judge a fit by `τ_f(ω)` over the range it was measured on,
not coefficient by coefficient.

Back-EMF *is* in here, in `k_v`: `DCMotor` has no back-EMF term, because a
constant-velocity experiment cannot separate it from speed-dependent friction. A
compensator built from this set must therefore skip `k_v` or it cancels the motor's own
damping.

Fit these with examples/friction_identification.jl, which prints a ready-to-paste
`withparams` call for the `friction_identified` set below.
"""
Base.@kwdef struct FrictionParams
    kc::Float64     = 0.0      # Coulomb torque                      [N·m]
    kv::Float64     = 2.6e-4   # first order: friction + back-EMF    [N·m·s/rad]
    k2::Float64     = 0.0      # quadratic                           [N·m·s²/rad²]
    k3::Float64     = 0.0      # cubic                               [N·m·s³/rad³]
    w_tanh::Float64 = 0.5      # sign-smoothing width, 0 = hard sign [rad/s]
end

"Copy of `p` with the named fields replaced (fill these from a fit)."
withparams(p::FrictionParams; changes...) =
    FrictionParams(; (f => get(changes, f, getfield(p, f)) for f in fieldnames(FrictionParams))...)

# Nominal: purely first-order, at the value the axis used to get from the old damper --
# `br + kt*km/Rm` = 5.0e-5 + 2.1e-4 with datasheet constants. It has to include the
# back-EMF now, since `DCMotor` no longer does; a `kv` of just `br` would leave the
# nominal plant six times less damped than the device. A fit adds the Coulomb and
# higher-order terms.
const friction_nominal = FrictionParams()

# The identified set, from a constant-velocity run on the QUBE (2026-07-30, 40 s sweep,
# 1..40 rad/s in both directions; 4875 of 8000 samples survived the constant-velocity
# selection, residual RMS 0.0105 V over a 4.01 V command range).
#
# `kv` here is friction *and* back-EMF together -- see `FrictionAndBackEMF` for why the
# two are not separated. That makes the set a direct restatement of what was measured
# (`tau = kt/Rm * u(w)`, no subtraction), so it is dissipative by construction: the
# measured command rises monotonically with speed, therefore so does the torque.
#
# `kc` is the number to treat with care in absolute terms -- it absorbs whatever deadband
# compensation the driver was applying during the run.
const friction_identified = withparams(friction_nominal;
    kc      = 0.00070810759,
    kv      = 0.00012323688,
    k2      = 3.5870033e-06,
    k3      = -4.0319332e-08,
)

# ---------------------------------------------------------------------------
## Friction-experiment log layout
# ---------------------------------------------------------------------------
# Shared by the `DataLogger` inside `FurutaFriction` (which writes the file), the
# generator that opens it, and the identification script that reads it back — so the
# column order is stated once. `time` is the program's own elapsed time, not the
# driver's wall clock: the row is written inside the tick that produced it.
const FRICTION_LOG_COLUMNS = ["time", "w_ref", "shoulder_angle", "shoulder_velocity",
                              "control_input", "elbow_angle"]
const FRICTION_LOG_HEADER = join(FRICTION_LOG_COLUMNS, "\t")
const FRICTION_LOG_NCOLS = length(FRICTION_LOG_COLUMNS)
const FRICTION_LOG_FILE = "friction_experiment.csv"