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

# NOTE: this is the fit from a run with the driver's deadband compensation at its default.
# The set fitted with compensation *off* (kc = 3.7e-3, i.e. 0.57 V of breakaway) was tried and
# reverted: it describes a plant no controller run uses, and the swing-up holds 1.9 deg off
# vertical on it instead of 0.2 deg. Replace the numbers below with a fresh fit on the restored
# configuration.
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

# ---------------------------------------------------------------------------
## Identification-experiment layout
# ---------------------------------------------------------------------------
# The open-loop replay `FurutaIdentification` performs to collect identification data. The first
# four columns are exactly the layout examples/pendulum_identification.jl and
# examples/analyze_discrimination.jl read
# (positionally, so nothing after column 4 disturbs them), which is what makes a run
# drop-in for those scripts. `u_des` is the designed sample before the safety supervisor saw
# it, and `tripped` the supervisor's latch, so the log says for itself where the supervisor
# intervened rather than leaving it to be inferred; then the loop diagnostics.
const IDENTIFICATION_LOG_COLUMNS = ["time", "shoulder_angle", "elbow_angle", "control_input",
                                    "u_des", "tripped", "dt", "exec"]
const IDENTIFICATION_LOG_HEADER = join(IDENTIFICATION_LOG_COLUMNS, "\t")
const IDENTIFICATION_LOG_NCOLS = length(IDENTIFICATION_LOG_COLUMNS)
const IDENTIFICATION_LOG_FILE = "identification_experiment.csv"
# What examples/input_design.jl writes: `time` then the designed voltage `u`.
const IDENTIFICATION_TRAJ_FILE = "input_design.csv"
const IDENTIFICATION_TRAJ_COLUMN = 2

# ---------------------------------------------------------------------------
## Swing-up run log layout
# ---------------------------------------------------------------------------
# Same arrangement for `FurutaHardware`: the `DataLogger` inside the program writes these
# columns, whichever target the program was built for, and the driver opens the file with
# them. The first four are what `plotD` expects (`readdlm(path, skipstart = 1)'`), then the
# loop diagnostics `dt`/`exec` and the raw encoder counts — the layout the C harness used to
# write from outside the program, kept identical so existing logs, plot sessions and readers
# are unaffected. Eight columns is also `log_row`'s arity, so this log is full.
const SWINGUP_LOG_COLUMNS = ["time", "shoulder_angle", "elbow_angle", "control_input",
                             "dt", "exec", "count_shoulder", "count_elbow"]
const SWINGUP_LOG_HEADER = join(SWINGUP_LOG_COLUMNS, "\t")
const SWINGUP_LOG_NCOLS = length(SWINGUP_LOG_COLUMNS)
# Relative on purpose: the exported C runs in its own directory on whatever machine it was
# deployed to, and an absolute path from this machine would be wrong there. The in-process
# runners resolve it against the analysis' `output_dir`.
const SWINGUP_LOG_FILE = "run_hardware.csv"
const FRICTION_LOG_FILE = "friction_experiment.csv"
# ---------------------------------------------------------------------------
## MPC swing-up run log layout
# ---------------------------------------------------------------------------
# What the `DataLogger` inside `FurutaMPCHardware` writes. The first six columns are the
# swing-up log's, so `plotD` and the readers of those logs work unchanged; the last two are
# what judges the MPC on the rig -- acados' status of every solve and whether the MPC was in
# command (1) or the energy swing-up (0) -- in place of the raw encoder counts.
const MPC_LOG_COLUMNS = ["time", "shoulder_angle", "elbow_angle", "control_input",
                         "dt", "exec", "exitflag", "stabilizing"]
const MPC_LOG_HEADER = join(MPC_LOG_COLUMNS, "\t")
const MPC_LOG_NCOLS = length(MPC_LOG_COLUMNS)
const MPC_LOG_FILE = "run_mpc.csv"

# The MPC's state signals, named as they appear in the compiled prediction model (the
# `qube` subsystem of `furuta_mpc_dynamics`'s system): the two joint angles and their
# derivatives, in the order the `FurutaMPC` component connects them to the MPC's state input
# -- [shoulder_angle, elbow_angle, shoulder_velocity, elbow_velocity]. The plant's
# `shoulder_angle`/`elbow_angle` outputs are these joint angles exactly, so the hardware
# measurements and the estimated velocities map onto the model states one to one.
const FURUTA_MPC_STATES = ["qube₊shoulder_joint₊phi", "qube₊elbow_joint₊phi",
                           "qube₊shoulder_joint₊phiˍt", "qube₊elbow_joint₊phiˍt"]
# The arm angle, the signal the end-stop constraint is placed on.
const FURUTA_MPC_ARM_SIGNAL = ["qube₊shoulder_joint₊phi"]
