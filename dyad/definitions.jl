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
    r_cm_r::Float64 = 0.085 / 2                             # rotary arm CoM radius
    Jr::Float64     = 0.095 * 0.085^2 / 3 - 0.095 * (0.085 / 2)^2   # arm body inertia
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
    Jr      = 3.4301897e-05,
    Jp      = 2.4501809e-05,
    br      = 0.00016757993,
    bp      = 3.0152273e-23,
    kt      = 0.054513092,
    mr      = 0.060555804,
    r_cm_r  = 0.033931805,
)