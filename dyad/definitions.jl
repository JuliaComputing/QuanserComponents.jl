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
    km::Float64     = 0.042                                 # back-EMF constant
    mr::Float64     = 0.095                                 # rotary arm mass (incl. encoder)
    r::Float64      = 0.085                                 # rotary arm length
    #r_cm_r::Float64 = 0.085 / 2                             # rotary arm CoM radius
    #Jr::Float64     = 0.095 * 0.085^2 / 3 - 0.095 * (0.085 / 2)^2   # arm body inertia
    br::Float64     = 0.05e-3                               # arm viscous damping
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
    Jp      = 2.5047706e-05,
    br      = 9.9316955e-05,
    bp      = 4.8190339e-06,
    kt      = 0.055684206,
    mr      = 0.026100653,
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

`k_v` is directly comparable with the plant's `br` ([`IdParams`](@ref)) — it is the
same mechanical viscous coefficient, which is why the fit subtracts the back-EMF
contribution `k_t k_m / R_m` that `QubePendulum` folds into its damper. `k_c` is the
Coulomb (breakaway) torque that `br` alone cannot represent, and the two
higher-order terms absorb the velocity dependence that is left over.

Fit these with examples/friction_identification.jl, which prints a ready-to-paste
`withparams` call for the `friction_identified` set below.
"""
Base.@kwdef struct FrictionParams
    kc::Float64     = 0.0      # Coulomb torque                      [N·m]
    kv::Float64     = 0.05e-3  # viscous, = IdParams.br              [N·m·s/rad]
    k2::Float64     = 0.0      # quadratic                           [N·m·s²/rad²]
    k3::Float64     = 0.0      # cubic                               [N·m·s³/rad³]
    w_tanh::Float64 = 0.5      # sign-smoothing width, 0 = hard sign [rad/s]
end

"Copy of `p` with the named fields replaced (fill these from a fit)."
withparams(p::FrictionParams; changes...) =
    FrictionParams(; (f => get(changes, f, getfield(p, f)) for f in fieldnames(FrictionParams))...)

# Nominal: pure viscous friction at the plant's nominal `br`, i.e. what
# `QubePendulum` already models. A fit should mostly add `kc`.
const friction_nominal = FrictionParams()

# The identified set. Replace with the output of examples/friction_identification.jl.
const friction_identified = friction_nominal

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