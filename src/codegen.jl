# Code generation for the discrete swing-up controller.
#
# The `Swingup` controller in `FurutaSwingup` is a purely discrete (clocked)
# subsystem: algebraic blocks plus discrete-time velocity estimators
# (`DiscreteDerivative` + `ExponentialFilter`). SynchToolkit's `stkcompile` can
# compile such a subsystem into a standalone synchronous node and emit Julia- or
# C-executable code for it.
#
# The generated node has the runtime signature
#     u = step(shoulder_angle, elbow_angle, tick, gains, auto)
# where `tick` is the periodic-clock trigger (`Bool`), `gains` carries the
# tunable LQR gains `L1..L4` and the stabilizer saturation `umax`, and `auto`
# carries the remaining model parameters (resolved to their model values). The
# single output is the control voltage.

using SynchToolkit
using SynchToolkit: ClockedInput, ClockedOutput, InputClock, ParametersStruct,
                    SynchExecutable
import SynchCompiler
import SynchJulia
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using DiscreteComponents: PeriodicClock
using OrderedCollections: OrderedDict

export build_discrete_controller, generate_swingup_controller, export_swingup_c,
       SwingupController

"""
    build_discrete_controller(; Ts=0.005, overrides...)

Build a purely discrete `System` wrapping the `Swingup` controller, driven by a
`PeriodicClock` at sample time `Ts`. The measured angles enter as clocked input
variables and the control voltage leaves as a clocked output.

The clock is linked to the controller through the equation
`clock.y ~ swingup.shoulder_angle`, which places the whole controller partition on
the periodic clock while leaving the two measured angles as free (input) signals.

Returns a named tuple with the `sys`, the `swingup` and `clock` subsystems, the
`Clock` object `clk`, and the resolved parameter value map `values` (parameter =>
`Float64`).

The controller uses the `Swingup` model's tuned defaults (energy-swingup gain,
arm-centering, LQR gains and saturations), which are set for the QuanserInterface-
matched `QubePendulum` plant. Pass `overrides` (e.g. `var"lqrstabilizer.umax" =>
...`) to change them.
"""
function build_discrete_controller(; Ts = 0.005, overrides...)
    @named swingup = SwingupWithHoming(; overrides...)
    @named clock = PeriodicClock(dt = Ts)
    sys = System([clock.y ~ swingup.shoulder_angle], t;
                 systems = [swingup, clock], name = :controller)

    # Resolve every parameter to a concrete number. Dyad components store values in
    # `default_values` (as `initial_conditions`), and some values are expressions of
    # other parameters (e.g. `energy.l => Lp/2`), so substitute to a fixed point.
    ss = ModelingToolkit.expand_connections(sys)
    dv = ModelingToolkit.default_values(ss)
    values = Dict{Any, Any}()
    for p in ModelingToolkit.parameters(ss)
        pu = ModelingToolkit.unwrap(p)
        values[pu] = _resolve_value(dv[pu], dv)
    end

    return (; sys, swingup, clock, clk = ModelingToolkit.Clock(Ts), values)
end

# Resolve a possibly-symbolic default to a concrete number (or vector of numbers for
# array parameters), substituting the default map to a fixed point.
function _resolve_value(v, dv)
    x = ModelingToolkit.unwrap(v)
    for _ in 1:20
        (x isa Number || x isa AbstractArray{<:Number}) && break
        x = ModelingToolkit.unwrap(ModelingToolkit.fixpoint_sub(x, dv))
    end
    xv = Symbolics.value(x)
    return xv isa AbstractArray ? Float64.(vec(collect(xv))) : Float64(xv)
end

"""
    generate_swingup_controller(; Ts=0.005, overrides...)

Compile the discrete swing-up controller to a SynchJulia node. Returns a named
tuple `(; topmod, controller, gain_syms)` where `topmod` is the evaluated module
(with `topmod.top`, `topmod.StaticGains`, `topmod.AutoPars`) and `controller` is
the [`build_discrete_controller`](@ref) result.

The node argument order is `(shoulder_angle::Float64, elbow_angle::Float64,
tick::Bool, gains::StaticGains, auto::AutoPars)`; the LQR gains `L1..L4` and the
stabilizer `umax` are the runtime-settable `StaticGains` fields, the rest are
baked into `AutoPars`.
"""
function generate_swingup_controller(; Ts = 0.005, kwargs...)
    c = build_discrete_controller(; Ts, kwargs...)
    lqr = c.swingup.runtime.swingup.lqrstabilizer
    gain_syms = OrderedDict{Any, Symbol}(
        ModelingToolkit.unwrap(lqr.L)   => :L,
        ModelingToolkit.unwrap(lqr.umax) => :umax,
    )
    inputs = [
        ClockedInput(c.swingup.shoulder_angle),
        ClockedInput(c.swingup.elbow_angle),
        InputClock(c.clk),
        ParametersStruct(; arg_name = :gains, struct_name = :StaticGains,
                           parameters = gain_syms, generated = false),
        ParametersStruct(; arg_name = :auto, struct_name = :AutoPars),
    ]
    outputs = [ClockedOutput(c.swingup.u)]
    topmod = SynchToolkit.stkcompile(c.sys; inputs, outputs)
    return (; topmod, controller = c, gain_syms)
end

"""
    SwingupController(; Ts=0.005, backend=:julia, L=nothing, umax=nothing, overrides...)

A ready-to-call runtime wrapper around the generated swing-up controller. Compiles
the controller, builds a `SynchExecutable` on `backend` (`:julia` or `:c`), and
populates the parameter structs. Call it as `u = controller(shoulder, elbow)` to
advance one control step (ticking the periodic clock) and obtain the voltage.

`L` overrides the LQR feedback gains `[L1, L2, L3, L4]` and `umax` the stabilizer
saturation; both default to the model's tuned values.
"""
struct SwingupController{E, G, A}
    exe::E
    gains::G
    auto::A
end

function SwingupController(; Ts = 0.005, backend::Symbol = :julia,
                            L = nothing, umax = nothing, kwargs...)
    gen = generate_swingup_controller(; Ts, kwargs...)
    return _make_runtime(gen; backend, L, umax)
end

# Build a runtime from an already-compiled controller (avoids recompiling).
# `stkcompile` evaluates a fresh module, so the generated `StaticGains`/`AutoPars`
# types and `top` node may be newer than the current world age; reach them through
# `invokelatest` so a one-shot `compile + build` in a single call still works.
function _make_runtime(gen; backend::Symbol = :julia, L = nothing, umax = nothing)
    topmod = gen.topmod
    node = Base.invokelatest(getproperty, topmod, :top)
    SG = Base.invokelatest(getproperty, topmod, :StaticGains)
    AP = Base.invokelatest(getproperty, topmod, :AutoPars)
    vals = gen.controller.values

    # AutoPars fields are named after the (namespaced) parameter symbols.
    sym_by_name = Dict(Symbol(string(p)) => ModelingToolkit.unwrap(p)
                       for p in keys(vals))
    apfields = Base.invokelatest(fieldnames, AP)
    apkw = Dict(f => vals[sym_by_name[f]] for f in apfields if haskey(sym_by_name, f))
    auto = Base.invokelatest(AP; apkw...)

    lqr = gen.controller.swingup.runtime.swingup.lqrstabilizer
    Ldef = vals[ModelingToolkit.unwrap(lqr.L)]                # model default (4-vector)
    Lv = L === nothing ? Ldef : collect(float.(L))
    umaxv = umax === nothing ? vals[ModelingToolkit.unwrap(lqr.umax)] : umax
    gains = Base.invokelatest(SG; L = Lv, umax = umaxv)

    exe = Base.invokelatest(SynchExecutable, node,
                            (Float64, Float64, Bool, typeof(gains), typeof(auto)); backend)
    return SwingupController(exe, gains, auto)
end

"""
    (c::SwingupController)(shoulder_angle, elbow_angle; tick=true) -> u

Advance the controller one step and return the control voltage. Angles are in
radians (shoulder/arm angle and pendulum angle; the pendulum-upright reference is
π, matching `QuanserInterface`).
"""
# The executable's step/reset dispatch on types generated by `stkcompile`'s `eval`,
# so route through `invokelatest` to stay correct even when a controller is compiled
# and used within the same world (e.g. one test block). The overhead is negligible
# for a control step, and the C backend (the performance path) does not run Julia here.
function (c::SwingupController)(shoulder_angle, elbow_angle; tick::Bool = true)
    out = Base.invokelatest(SynchJulia.step!, c.exe, Float64(shoulder_angle),
                            Float64(elbow_angle), tick, c.gains, c.auto)
    return only(values(out))
end

SynchToolkit.reset!(c::SwingupController) = Base.invokelatest(SynchToolkit.reset!, c.exe)

"""
    export_swingup_c(dir; Ts=0.005, overrides...)

Generate the swing-up controller and export standalone C sources into `dir`
(`top.c`, `top.h`, `top.pc`, `synchjulia.h`). Returns `(; dir, topmod, mangled)`
where `mangled` is the base symbol name for the emitted `<mangled>_step` /
`<mangled>_reset` functions.
"""
function export_swingup_c(dir; Ts = 0.005, kwargs...)
    gen = generate_swingup_controller(; Ts, kwargs...)
    node = Base.invokelatest(getproperty, gen.topmod, :top)
    SG = Base.invokelatest(getproperty, gen.topmod, :StaticGains)
    AP = Base.invokelatest(getproperty, gen.topmod, :AutoPars)
    Base.invokelatest(SynchCompiler.export_c, dir, node)
    mangled = SynchCompiler.mangle("top", Float64, Float64, Bool, SG, AP)
    return (; dir, gen.topmod, mangled)
end
