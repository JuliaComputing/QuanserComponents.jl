# Hardware I/O from inside a synchronous program.
#
# `HardwareMeasurement` and `HardwareCommand` (dyad/hardware_loop.dyad) do their
# encoder read and amplifier write *inside* the compiled node rather than in the
# surrounding real-time loop. That needs two things a synchronous dataflow
# program does not normally have: a way to call arbitrary Julia, and a place to
# keep a device handle.
#
# Both come from `@register_symbolic`: the operators below are opaque to the
# Lustre translator, which emits them verbatim into the generated code
# (SynchToolkit's `build_rhs_op` fallback), and the device handle rides along as
# an argument of type `HardwareIO`. `HardwareIO` is mutable, so SynchToolkit
# classifies it as an opaque reference type and passes it through untouched, and
# its identity is preserved through compilation -- mutating it from outside is
# how the hardware gets bound after the program has been compiled.
#
# Two properties are load-bearing and are asserted by the tests:
#
#  1. **Exactly one read and one write per tick.** `stkcompile` does no CSE, no
#     DCE and no observed-variable inlining, so one equation is one call.
#  2. **Ordering.** Equation scheduling only respects data dependencies, so the
#     cache reads take a `dep` argument that carries no information other than
#     "the read has happened". Without it they could be scheduled before the
#     read and serve the previous tick's values. (Same trick as
#     `DiscreteComponents`' `seeded_rand(seed, td)`.)
#
# This is a Julia-backend-only mechanism: the C backend cannot cross-compile a
# call into Julia. The plain `SwingupController` and `export_swingup_c` in
# codegen.jl are unaffected and remain the route to standalone C.

using ModelingToolkit

export HardwareIO, bind_hardware!

"""
    HardwareIO(; measure = () -> (0.0, 0.0), control = u -> nothing)

Device handle passed into a compiled controller that performs its own I/O.

`measure` is called once per tick and must return the two measured angles
`(shoulder, elbow)` in radians (anything indexable works, e.g. the `Vector`
returned by `QuanserInterface.measure`). `control` is called once per tick with
the clamped voltage.

Keeping these as fields rather than calling `QuanserInterface` directly is what
lets this package stay independent of the HIL stack; bind them with
[`bind_hardware!`](@ref).

The remaining fields are written by the controller and are readable afterwards:
`shoulder`, `elbow` and `u` hold the most recent values, and `n_measure` /
`n_control` count the calls made (used to assert one call per tick).
"""
mutable struct HardwareIO
    measure::Any
    control::Any
    shoulder::Float64
    elbow::Float64
    u::Float64
    n_measure::Int
    n_control::Int
end

HardwareIO(; measure = () -> (0.0, 0.0), control = u -> nothing) =
    HardwareIO(measure, control, 0.0, 0.0, 0.0, 0, 0)

"""
    bind_hardware!(io::HardwareIO; measure, control) -> io

Point `io` at a device and clear its cache and call counters. Typical use with
QuanserInterface:

```julia
bind_hardware!(io; measure = () -> QuanserInterface.measure(process),
                   control = u -> QuanserInterface.control(process, [u]))
```

Safe to call on an `io` that is already bound into a compiled controller: the
controller holds the same object, so the new device takes effect immediately
without recompiling.
"""
function bind_hardware!(io::HardwareIO; measure = io.measure, control = io.control)
    io.measure = measure
    io.control = control
    reset_hardware!(io)
end

"""
    reset_hardware!(io::HardwareIO) -> io

Clear the cached angles, the last command and the call counters. `SynchToolkit.reset!`
resets the node's own state but knows nothing about `io`, so reset both when
restarting an experiment.
"""
function reset_hardware!(io::HardwareIO)
    io.shoulder = 0.0
    io.elbow = 0.0
    io.u = 0.0
    io.n_measure = 0
    io.n_control = 0
    io
end

# The registered operators. `HardwareIO` must already be defined at this point:
# `@register_symbolic` evaluates the argument types at macro-expansion time.
# The return types are annotated because SynchCompiler rejects a call whose
# inferred return type is not concrete.
@register_symbolic hw_measure!(io::HardwareIO)::Real
@register_symbolic hw_shoulder(io::HardwareIO, dep::Real)::Real
@register_symbolic hw_elbow(io::HardwareIO, dep::Real)::Real
@register_symbolic hw_write(io::HardwareIO, u::Real, umax::Real)::Real

"""
    hw_measure!(io) -> shoulder

The one effectful read: sample both encoders, cache them on `io` and return the
shoulder angle. The returned value doubles as the dependency token that orders
[`hw_shoulder`](@ref) and [`hw_elbow`](@ref) after this call.
"""
function hw_measure!(io::HardwareIO)::Float64
    y = io.measure()
    io.shoulder = y[1]
    io.elbow = y[2]
    io.n_measure += 1
    io.shoulder
end

"Cached shoulder angle. `dep` only forces this to be scheduled after `hw_measure!`."
hw_shoulder(io::HardwareIO, dep::Real)::Float64 = io.shoulder

"Cached elbow angle. `dep` only forces this to be scheduled after `hw_measure!`."
hw_elbow(io::HardwareIO, dep::Real)::Float64 = io.elbow

"""
    hw_write(io, u, umax) -> u_applied

Clamp `u` to `[-umax, umax]`, write it to the amplifier and return what was
applied. Ordered after the read because `u` depends on the measured angles.
"""
function hw_write(io::HardwareIO, u::Real, umax::Real)::Float64
    uc = clamp(Float64(u), -Float64(umax), Float64(umax))
    io.u = uc
    io.n_control += 1
    io.control(uc)
    uc
end
