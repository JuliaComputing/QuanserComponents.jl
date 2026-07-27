# Runtime for `FurutaHardware`: the swing-up controller with the encoder read and the
# amplifier write inside the synchronous program.
#
# Compared with `generate_swingup_controller` (codegen.jl), the two measured angles are no
# longer `ClockedInput`s -- `HardwareMeasurement` produces them itself -- so the node's only
# input is the clock tick, and the device handle rides in through an extra static parameter
# struct. The node returns the two measured angles and the applied voltage, which is
# everything the caller needs for logging.
#
# What is left outside the program is only timing and logging:
#
#     ctrl = HardwareSwingupController(; Ts)
#     bind_hardware!(ctrl; measure = () -> QuanserInterface.measure(process),
#                          control = u -> QuanserInterface.control(process, [u]))
#     for i in 1:N
#         @periodically Ts begin
#             out = ctrl()
#             push!(data, [time() - t0, out.shoulder, out.elbow, out.u])
#         end
#     end
#
# Julia backend only. `SwingupController`/`export_swingup_c` remain the route to
# standalone C, and this node must never be built with `backend = :c`: the C backend
# would have to cross-compile the Julia calls in `HardwareMeasurement`/`HardwareCommand`.

export generate_hardware_controller, HardwareSwingupController

"""
    generate_hardware_controller(; Ts=0.005, L=nothing, umax=nothing, overrides...)

Compile `FurutaHardware` to a SynchJulia node that performs its own hardware I/O.

Returns a named tuple `(; topmod, Ldef, umaxdef)` in the same shape as
[`generate_swingup_controller`](@ref), where `topmod` is the evaluated runtime module. The
node argument order is `(tick::Bool, gains::StaticGains, auto::AutoPars, hw::HardwareParams)`
and it returns `(shoulder_angle, elbow_angle, u_applied)`.

`hw` carries the two `HardwareIO` handles (one per hardware component); they have no
defaults, so they must be supplied when the struct is constructed. `L`, `umax` and
`overrides` behave as in `generate_swingup_controller`.
"""
function generate_hardware_controller(; Ts = 0.005, L = nothing, umax = nothing, overrides...)
    @named model = FurutaHardware(; Ts, overrides...)
    lqr_stab = model.swingup.runtime.swingup.lqrstabilizer
    # Dyad parameter values live in `initial_conditions` (not `getdefault`), and the
    # generated constructor does not forward `initial_conditions`, so wrap the model in a
    # trivial parent to override the stabilizer gain/saturation there.
    ics = Dict{Any, Any}()
    L    === nothing || (ics[lqr_stab.L]    = collect(float.(L)))
    umax === nothing || (ics[lqr_stab.umax] = float(umax))
    sys = System(Equation[], t; systems = [model], name = :hwcontroller,
                 initial_conditions = ics)

    Lsym, usym, Ldef, umaxdef = _resolve_stabilizer_gains(sys, lqr_stab)

    gain_syms = OrderedDict{Any, Symbol}(Lsym => :L, usym => :umax)
    hw_syms = OrderedDict{Any, Symbol}(
        ModelingToolkit.unwrap(model.measurement.io) => :meas_io,
        ModelingToolkit.unwrap(model.command.io)     => :cmd_io,
    )
    inputs = [
        InputClock(ModelingToolkit.Clock(Ts)),
        ParametersStruct(; arg_name = :gains, struct_name = :StaticGains,
                           parameters = gain_syms, generated = false),
        ParametersStruct(; arg_name = :auto, struct_name = :AutoPars),
        ParametersStruct(; arg_name = :hw, struct_name = :HardwareParams,
                           parameters = hw_syms, generated = false),
    ]
    # Output order defines the fields of what the controller returns. Neither `name` nor
    # `clock` may be passed here: `name` desynchronises the declared and assigned Lustre
    # names, and `clock` hits a missing branch in SynchToolkit's `build_output`.
    outputs = [
        ClockedOutput(model.measurement.shoulder_angle),
        ClockedOutput(model.measurement.elbow_angle),
        ClockedOutput(model.command.u_applied),
    ]
    topmod = SynchToolkit.stkcompile(sys; inputs, outputs)
    return (; topmod, Ldef, umaxdef)
end

"""
    HardwareSwingupController(; Ts=0.005, L=nothing, umax=nothing, io=HardwareIO(), overrides...)

A ready-to-call runtime wrapper around the generated hardware controller. Compiles
`FurutaHardware`, builds a `SynchExecutable` on the Julia backend and populates the
parameter structs with `io`.

Advance one control step with `out = controller()`, which returns
`(; shoulder, elbow, u)`: the two angles the program measured and the voltage it applied.
Bind the device with [`bind_hardware!`](@ref) before the first step, and use
`SynchToolkit.reset!` to restart from the homing state (it resets `io` as well).
"""
struct HardwareSwingupController{E, G, A, H}
    exe::E
    gains::G
    auto::A
    hw::H
    io::HardwareIO
end

function HardwareSwingupController(; Ts = 0.005, L = nothing, umax = nothing,
                                     io::HardwareIO = HardwareIO(), kwargs...)
    gen = generate_hardware_controller(; Ts, kwargs...)
    return _make_hardware_runtime(gen; L, umax, io)
end

# As in `_instantiate` (codegen.jl), `stkcompile` evaluates the runtime module in a newer
# world than this frame, so the boundary is crossed exactly once, here: the single
# `@invokelatest` raises the world for every module member read and constructor call, and
# everything returned is world-safe to use from any frame afterwards. `AutoPars` takes both
# static structs, in the order they appear in `inputs`.
function _make_hardware_runtime(gen; L = nothing, umax = nothing,
                                  io::HardwareIO = HardwareIO())
    Lv = collect(float.(something(L, gen.Ldef)))
    umaxv = float(something(umax, gen.umaxdef))
    function consume(m)
        gains = m.StaticGains(; L = Lv, umax = umaxv)
        hw = m.HardwareParams(; meas_io = io, cmd_io = io)
        auto = m.AutoPars(gains, hw)
        exe = m.executable(:julia)
        return HardwareSwingupController(exe, gains, auto, hw, io)
    end
    return Base.@invokelatest consume(gen.topmod)
end

"""
    (c::HardwareSwingupController)(; tick=true) -> (; shoulder, elbow, u)

Advance the controller one step: it reads the encoders, runs the state machine and writes
the motor voltage, then reports what it measured and applied. Angles are in radians, with
the pendulum-upright reference at π (matching `QuanserInterface`).

With `tick=false` the clock does not fire, so no hardware is touched and the returned
values are meaningless (`nothing`).
"""
function (c::HardwareSwingupController)(; tick::Bool = true)
    out = SynchJulia.step!(c.exe, tick, c.gains, c.auto, c.hw)
    v = values(out)
    return (; shoulder = v[1], elbow = v[2], u = v[3])
end

function SynchToolkit.reset!(c::HardwareSwingupController)
    SynchToolkit.reset!(c.exe)
    reset_hardware!(c.io)
    return c
end

"""
    bind_hardware!(c::HardwareSwingupController; measure, control)

Point the controller's `HardwareIO` at a device. See [`bind_hardware!(::HardwareIO)`](@ref).
"""
bind_hardware!(c::HardwareSwingupController; kwargs...) = bind_hardware!(c.io; kwargs...)
