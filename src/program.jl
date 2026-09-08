# Building and running a synchronous program for the QUBE rig, whichever program it is.
#
# Every program in this library -- `FurutaHardware` (swing-up), `FurutaFriction` (friction
# experiment), `FurutaIdentification` (open-loop replay), `FurutaMPCHardware` and
# `FurutaMPCMultirateHardware` (MPC) -- is a purely discrete model that contains its own hardware
# I/O and its own logging, so compiling one and running it is the same job every time: build the
# model with the overrides applied, resolve the parameters that are to stay runtime-settable,
# `stkcompile` it, construct the parameter structs, then tick it against the device with the log
# open. What differs from program to program -- which parameters are tunable, which outputs come
# back, what the log columns mean -- is declared once per model in a [`ProgramSpec`](@ref), which
# [`program_spec`](@ref) returns for the model's constructor.
#
# There are three ways to put a program on the rig, and each is named for what it is:
#
#   - [`run_inprocess!`](@ref) ticks a compiled node from a Julia timing loop in this process, on
#     either SynchJulia backend. The only route for the MPC programs, whose prediction model has no
#     symbolic form to render to C.
#   - `run_c!` (harness.jl) exports the node as standalone C, builds it here or on another machine
#     and runs that binary.
#   - [`run_ode!`](@ref) does not compile a node at all: an ODE solver steps the model, paced by the
#     model itself, so that every clocked variable ends up in a solution object.
#
# The per-program files (codegen.jl, friction.jl, identification.jl, mpc.jl) hold nothing but the
# model's `ProgramSpec` and what the model is about; if something in one of them is not about
# *that* program, it belongs here.

using SynchToolkit
using SynchToolkit: ClockedOutput, InputClock, ParametersStruct
import SynchJulia
using ModelingToolkit
using ModelingToolkit: t_nounits as t
using DiscreteComponents: PeriodicClock
using OrderedCollections: OrderedDict
# `default_values` (a system's bindings, initial conditions and observed equations merged
# into one map) is owned by SymbolicIndexingInterface; ModelingToolkit no longer re-exports it.
using SymbolicIndexingInterface: default_values

export ProgramSpec, program_spec, ProgramLog, ProgramTrajectory, CompiledProgram,
       compile_program, build_parameter_structs, ProgramRuntime, run_inprocess!, run_ode!,
       read_log

# ---------------------------------------------------------------------------
## What a program declares about itself
# ---------------------------------------------------------------------------
"""
    ProgramSpec(; name, tunables, outputs, output_names, log, traj=nothing, backends=(:julia, :c),
                  prerequisites, check, ode_kwargs=nothing, ode_warmup=nothing)

What [`compile_program`](@ref) and the run functions need to know about a model beyond its
constructor. One is defined per program by a method of [`program_spec`](@ref) on the model's
constructor type.

  - `name`: the system name the model is built with.
  - `tunables`: model parameter => field name, for the runtime-settable `TuningGains` struct. Each
    parameter is given as a function of the un-namespaced system (`nsys -> nsys.umax`), and has to
    be *unbound* -- see [`resolve_tunables`](@ref).
  - `outputs`: a function of the un-namespaced system returning the signals the node exposes, in
    the order [`ProgramRuntime`](@ref) reports them under `output_names`.
  - `log`: builds the [`ProgramLog`](@ref) the model's `DataLogger` writes, `log(file)` or
    `log()` for the program's default file.
  - `traj`: builds the [`ProgramTrajectory`](@ref) the program replays from the model's keyword
    arguments (`traj(kw::Dict)`), or `nothing` for a program that replays nothing.
  - `backends`: the SynchJulia backends the program can run on. The MPC programs are `(:julia,)`.
  - `prerequisites`: called before the model is built; the place to check for compiler features
    the program needs and fail with a message rather than deep inside `stkcompile`.
  - `check`: called with the loop period and the resolved tunable values whenever parameter
    structs are built, for warnings about values that compile but do not work.
  - `ode_kwargs`: the constructor keywords the model has to be built with for [`run_ode!`](@ref)
    -- what makes it pace itself in real time and record what that route is run for -- or
    `nothing` for a model that has no such mode.
  - `ode_warmup`: a function of the compiled system returning the parameter overrides that make
    the model read the device without writing to it, for the warm-up run of [`run_ode!`](@ref);
    `nothing` if the model cannot be clamped that way.
"""
struct ProgramSpec
    name::Symbol
    tunables::OrderedDict{Any, Symbol}
    outputs::Any
    output_names::Tuple{Vararg{Symbol}}
    log::Any
    traj::Any
    backends::Tuple{Vararg{Symbol}}
    prerequisites::Any
    check::Any
    ode_kwargs::Union{Nothing, NamedTuple}
    ode_warmup::Any
end

function ProgramSpec(; name::Symbol, tunables::AbstractDict, outputs, output_names,
                     log, traj = nothing, backends = (:julia, :c),
                     prerequisites = () -> nothing, check = (Ts, vals) -> nothing,
                     ode_kwargs = nothing, ode_warmup = nothing)
    return ProgramSpec(name, OrderedDict{Any, Symbol}(tunables), outputs,
                       Tuple(output_names), log, traj, Tuple(backends), prerequisites, check,
                       ode_kwargs, ode_warmup)
end

"""
    program_spec(ctor) -> ProgramSpec

The [`ProgramSpec`](@ref) of the program whose model `ctor` constructs. Defined for
`FurutaHardware`, `FurutaFriction`, `FurutaIdentification`, `FurutaMPCHardware` and
`FurutaMPCMultirateHardware`; a new program adds a method here and nothing else.
"""
program_spec(ctor) =
    throw(ArgumentError("$ctor is not one of this package's programs: no `program_spec` method \
                         is defined for it"))

# ---------------------------------------------------------------------------
## Runtime-settable parameters
# ---------------------------------------------------------------------------
"""
    resolve_tunables(sys, syms) -> OrderedDict{Symbol, Any}

Resolve the model's own values for the parameters that are to become a runtime-settable
`ParametersStruct`, keyed by the field name each takes in it.

A static `ParametersStruct` (`generated = false`) carries no defaults of its own -- unlike
the autogenerated `AutoPars`, whose field defaults SynchToolkit resolves from the system --
so the values have to be dug out here. `evaluate_varmap!` substitutes the default map into
itself to a fixed point, which is what makes a default that is an expression of other
parameters (`Ni` in terms of `Ti`, say) come out as a number.

Resolution happens against the full pre-partition default map on purpose: an expression
default may reference a parameter that the compiled partition drops, which the partitioned
map no longer has (SynchToolkit#144).

Every symbol handed in must be *unbound* -- a parameter nothing else binds -- or
SynchToolkit rejects the struct. In practice that means a root-level parameter, bound
downwards into the components that use it with Dyad's `final`, which is also what lets an
analysis forward its own value into it.
"""
function resolve_tunables(sys, syms::AbstractDict)
    SymT = ModelingToolkit.SymbolicT
    keysyms = [ModelingToolkit.unwrap(k) for k in keys(syms)]
    dv = Dict{SymT, SymT}(default_values(ModelingToolkit.expand_connections(sys)))
    ModelingToolkit.evaluate_varmap!(dv, keysyms)
    out = OrderedDict{Symbol, Any}()
    for (k, field) in syms
        v = Symbolics.value(dv[ModelingToolkit.unwrap(k)])
        out[field] = v isa AbstractArray ?
                     Float64.(vec(collect(Symbolics.value.(v)))) : Float64(v)
    end
    return out
end

# The Dyad compiler turns an analysis' `model = Foo(final x = x)` into a
# `Dict{SymbolicT, SymbolicT}` of un-namespaced model parameter => value, handed to the
# implementation as `spec.overrides`. Translate it into the keyword form the generated model
# constructor takes, so the analysis' parameters reach the model through the model's own
# entry point instead of the implementation reading them back off the spec. Nested paths
# arrive as `a₊b` and become Dyad's `a__b` override syntax.
_model_kwargs(::Nothing) = Dict{Symbol, Any}()
function _model_kwargs(overrides)
    kw = Dict{Symbol, Any}()
    for (k, v) in overrides
        kw[Symbol(replace(string(k), "₊" => "__"))] = Symbolics.value(v)
    end
    return kw
end

# ---------------------------------------------------------------------------
## The log a program writes
# ---------------------------------------------------------------------------
"""
    ProgramLog(file, columns)

Where a program's `DataLogger` writes and what the columns are.

Carried alongside the compiled program because three parties have to agree on it and only
one of them is the model: the component writes the rows, the driver opens the file
([`open_log!`](@ref) needs the header and the column count), and the exported C harness
opens it with the same three values baked into its config header. `file` is what the model
was built with, so it cannot drift from what the program actually writes.
"""
struct ProgramLog
    file::String
    columns::Vector{String}
end

header(l::ProgramLog) = join(l.columns, "\t")
ncols(l::ProgramLog) = length(l.columns)

open_log!(l::ProgramLog) = open_log!(l.file; header = header(l), ncols = ncols(l))

"""
    ProgramTrajectory(file, column)

A recorded input sequence a program replays, and which column of the file holds it.

The input-side counterpart of [`ProgramLog`](@ref), and carried for the same reason: the
`TrajectorySource` component reads the samples, the driver opens the file
([`open_traj!`](@ref)), and the exported C harness opens it with the same two values baked
into its config header. A program that replays nothing has `nothing` here.
"""
struct ProgramTrajectory
    file::String
    column::Int
end

open_traj!(t::ProgramTrajectory) = open_traj!(t.file; column = t.column)
open_traj!(::Nothing) = 0

# ---------------------------------------------------------------------------
## The model's clocks
# ---------------------------------------------------------------------------
"""
    model_clocks(sys) -> Vector{PeriodicClock}

The periodic clocks a model declares, fastest first.

Read off the built system rather than restated by the caller: a `PeriodicClock` component puts
its clock in the time-domain metadata of its output, so the clocks are known before any
inference runs. Clock identity is field-wise on `(dt, phase)`, so using the model's own clock
objects for the node's `InputClock`s cannot name a clock no equation lives on, which a period
written a second time could do by differing in the last bit.
"""
function model_clocks(sys)
    PC = ModelingToolkit.SciMLBase.PeriodicClock
    clocks = Set{PC}()
    for v in ModelingToolkit.unknowns(ModelingToolkit.expand_connections(sys))
        d = ModelingToolkit.getmetadata(ModelingToolkit.unwrap(v),
                                        ModelingToolkit.VariableTimeDomain, nothing)
        d isa PC && push!(clocks, d)
    end
    return sort!(collect(clocks); by = c -> c.dt)
end

# Each clock's period as an integer multiple of the fastest one, fastest first, so `(1,)` for a
# single-rate program. The node takes one boolean per clock and `ProgramRuntime` raises each
# every so many ticks of the fastest, which is only right if the periods are integer multiples
# and the clocks all start together.
function clock_divisors(clocks)
    isempty(clocks) &&
        throw(ArgumentError("the model declares no periodic clock; a program needs at least one"))
    base = first(clocks).dt
    all(c -> c.phase == 0, clocks) ||
        throw(ArgumentError("the model's clocks have to start together (phase 0); got \
                             $(clocks)"))
    divisors = map(clocks) do c
        ratio = c.dt / base
        d = round(Int, ratio)
        abs(ratio - d) <= 1e-9 * d ||
            throw(ArgumentError("the period $(c.dt) of one of the model's clocks is not an \
                                 integer multiple of the fastest, $base (ratio $ratio); the \
                                 program ticks the fastest clock and raises the others every \
                                 so many ticks of it"))
        d
    end
    return Tuple(divisors)
end

# ---------------------------------------------------------------------------
## Compiling
# ---------------------------------------------------------------------------
"""
    CompiledProgram

What [`compile_program`](@ref) returns: the model's [`ProgramSpec`](@ref), SynchToolkit's
`CompiledNode` in `compiled`, the two `ParametersStruct`s (kept because they double as the
constructors for the structs they were compiled into), the model's resolved values of the
tunable parameters in `tuning_defaults`, the [`ProgramLog`](@ref) and [`ProgramTrajectory`](@ref)
the model was built with, the loop period `Ts`, and the `divisors` of the model's clocks.

`Ts` is the period of the model's *fastest* clock, which is the one a driver ticks; for a
single-rate program that is the model's `Ts`. `divisors` gives every clock's period as an integer
multiple of it, fastest first, so `(1,)` for a single-rate program and `(1, 8)` for the multirate
MPC.
"""
struct CompiledProgram{C, TS, AS}
    spec::ProgramSpec
    compiled::C
    tuning_struct::TS
    auto_struct::AS
    tuning_defaults::OrderedDict{Symbol, Any}
    log::ProgramLog
    traj::Union{Nothing, ProgramTrajectory}
    Ts::Float64
    divisors::Tuple{Vararg{Int}}
end

"""
    compile_program(ctor; log_file=nothing, param_overrides=nothing, overrides...) -> CompiledProgram

Compile one of the rig's programs to a SynchJulia node. This is where the stkcompile call lives.

`ctor` is the model constructor -- `FurutaHardware`, `FurutaFriction`, `FurutaIdentification`,
`FurutaMPCHardware` or `FurutaMPCMultirateHardware` -- and everything else about the program
comes from its [`program_spec`](@ref). `overrides...` are passed to the constructor: the model's
own structural parameters (`Ts`, `Np`, `traj_file`) and Dyad `__`-separated paths into its
components, e.g. `control_system__energy_weight = 3e4`. `param_overrides` is an analysis'
`spec.overrides`, the same thing in the form the Dyad compiler produces. `log_file` is where the
model's `DataLogger` writes; it goes to the model, not just to the driver, so the component that
writes the file and the call that opens it cannot disagree about which file that is.

The node's argument order is `(ticks::Bool..., gains::TuningGains, auto::AutoPars)` -- one
boolean per clock of the model, fastest first. The clocks are read off the built model
([`model_clocks`](@ref)), so a multirate model needs nothing declared here: `divisors` in the
result records each clock's period as an integer multiple of the fastest, and
[`ProgramRuntime`](@ref) generates the tick pattern from it so that a caller ticks once per
`Ts`.
"""
function compile_program(ctor; log_file = nothing, param_overrides = nothing, overrides...)
    spec = program_spec(ctor)
    spec.prerequisites()
    # Every C library the program calls into has to exist before the `:c` backend links them
    # (the Julia backend only needs them at call time).
    ensure_qube_hw()
    ensure_qube_log()
    kw = Dict{Symbol, Any}(overrides)
    merge!(kw, _model_kwargs(param_overrides))
    log = log_file === nothing ? spec.log() : spec.log(log_file)
    traj = spec.traj === nothing ? nothing : spec.traj(kw)
    traj === nothing || ensure_qube_traj()
    sys = ctor(; name = spec.name, log_file = log.file, kw...)
    clocks = model_clocks(sys)
    divisors = clock_divisors(clocks)
    # `sys` is the root, so its own name is not part of the flattened symbol names; reach
    # for symbols through the un-namespaced view, which is what `default_values` and
    # `stkcompile` see.
    nsys = ModelingToolkit.toggle_namespacing(sys, false)
    tuning_syms = OrderedDict{Any, Symbol}(
        ModelingToolkit.unwrap(f(nsys)) => field for (f, field) in spec.tunables)
    tuning_defaults = resolve_tunables(sys, tuning_syms)
    tuning_struct = ParametersStruct(; arg_name = :gains, struct_name = :TuningGains,
                                      parameters = tuning_syms, generated = false)
    auto_struct = ParametersStruct(; arg_name = :auto, struct_name = :AutoPars)
    # One `InputClock` per clock of the model, in the order the node's `step` will take them as
    # boolean arguments. They are deliberately unnamed -- `InputClock(clk; name = ...)` reaches
    # `insert_clock(::LustreTranslator, ...)`, which has no method on this SynchToolkit (fixed
    # upstream by SynchToolkit.jl#189).
    inputs = SynchToolkit.Argument[InputClock(c) for c in clocks]
    push!(inputs, tuning_struct)
    push!(inputs, auto_struct)
    # Neither `name` nor `clock` may be passed to `ClockedOutput`: `name` desynchronises the
    # declared and assigned Lustre names, and `clock` hits a missing branch in
    # SynchToolkit's `build_output`. So the outputs are indexed positionally.
    outs = [ClockedOutput(o) for o in spec.outputs(nsys)]
    @info "Running stkcompile"
    compiled = SynchToolkit.stkcompile(sys; inputs, outputs = outs)
    return CompiledProgram(spec, compiled, tuning_struct, auto_struct, tuning_defaults, log,
                           traj, first(clocks).dt, divisors)
end

# ---------------------------------------------------------------------------
## The parameter structs
# ---------------------------------------------------------------------------
"""
    build_parameter_structs(gen::CompiledProgram; gains=(;)) -> (; gains, auto)

Construct the `TuningGains` and `AutoPars` parameter objects of a compiled program, with `gains`
overriding the model's resolved values field by field (a `nothing` leaves the model's value).

These objects are what the node's `step` is handed, in this process by [`ProgramRuntime`](@ref)
and as raw bytes in the exported C by `export_program_c`: their in-memory field bytes are
exactly what the exported C reads at its baked-in `fieldoffset`s. The spec's `check` runs on
the final values, so a warning about an unworkable tunable is given whichever way the program
is run.

`stkcompile` evaluates a runtime module in a newer world than this frame, but nothing here has
to know that: a `ParametersStruct` is callable with the `CompiledNode`, and does its own
`invoke_in_world` inside (SynchToolkit#159).
"""
function build_parameter_structs(gen::CompiledProgram; gains = (;))
    vals = OrderedDict{Symbol, Any}(gen.tuning_defaults)
    for (field, v) in pairs(gains)
        v === nothing && continue
        haskey(vals, field) ||
            throw(ArgumentError("$field is not one of this program's tunable parameters \
                                 ($(join(keys(vals), ", ")))"))
        vals[field] = vals[field] isa AbstractVector ? collect(float.(v)) : float(v)
    end
    gen.spec.check(gen.Ts, vals)
    cn = gen.compiled
    g = gen.tuning_struct(cn; vals...)
    # Pass the static struct: AutoPars defaults may be expressions of its fields.
    auto = gen.auto_struct(cn, g)
    return (; gains = g, auto)
end

# ---------------------------------------------------------------------------
## The runtime
# ---------------------------------------------------------------------------
"""
    ProgramRuntime(gen::CompiledProgram; backend=:julia, gains...)

A compiled program with its parameter structs and a `SynchExecutable` on `backend`, ready to be
ticked in this process.

`out = runtime()` advances one step: the program reads the encoders, computes, writes the
motor and appends a row to its log, and `out` is a `NamedTuple` of the node's outputs under
the spec's `output_names`. With `tick = false` the clock does not fire, so no hardware is
touched, no row is logged, and the values are meaningless. For a multirate program one call
is one tick of the *fastest* clock; the other clocks fire every so many of them, and an output
on a clock that did not fire comes back as `nothing`.

`gains` are the runtime-settable parameters by their `TuningGains` field name
(`ProgramRuntime(gen; umax = 5.0)`), overriding the model's values without a recompile; a
name that is not one of the program's tunables is an error. `backend` is `:julia` or `:c`, and
has to be one the spec allows.

Point the runtime at a device with [`open_hardware!`](@ref) or at a simulator with
[`bind_hardware!`](@ref) and open the log with [`open_log!`](@ref) to tick it by hand;
[`run_inprocess!`](@ref) does all of that, and the timing, for a real run.
"""
struct ProgramRuntime{names, K, E, G, A}
    exe::E
    gains::G
    auto::A
    log::ProgramLog
    traj::Union{Nothing, ProgramTrajectory}
    Ts::Float64
    # Each clock's period as an integer multiple of `Ts`, fastest first, so `(1,)` for a
    # single-rate program. A type parameter as well as a field so the tick pattern below is
    # built without allocating -- `run_inprocess!` runs with the collector off.
    divisors::NTuple{K, Int}
    counter::Base.RefValue{Int}
end

ProgramRuntime{names}(exe::E, gains::G, auto::A, log, traj, Ts,
                      divisors::NTuple{K, Int} = (1,)) where {names, K, E, G, A} =
    ProgramRuntime{names, K, E, G, A}(exe, gains, auto, log, traj, Float64(Ts), divisors, Ref(0))

function ProgramRuntime(gen::CompiledProgram; backend::Symbol = :julia, gains...)
    backend in gen.spec.backends ||
        throw(ArgumentError("the program $(gen.spec.name) runs on the \
                             $(join(repr.(gen.spec.backends), ", ")) backend only, not \
                             $(repr(backend)); see its `program_spec`"))
    p = build_parameter_structs(gen; gains)
    exe = SynchJulia.SynchExecutable(gen.compiled; backend)
    return ProgramRuntime{gen.spec.output_names}(exe, p.gains, p.auto, gen.log, gen.traj,
                                                 gen.Ts, gen.divisors)
end

# A held executable keeps stepping the code it was built with (SynchJulia >= 0.4), so
# `step!`/`reset!` are world-safe from any frame -- no `invokelatest` in the hot path.
#
# The subclock tick pattern is generated here rather than asked of the caller. A node with two
# clocks takes one boolean per clock, and raising the slow one without the fast one compiles and
# runs -- silently on the previous tick's measurements, since `Latest` returns the newest value
# the source clock produced and a simultaneous source tick is what makes the crossing free. So
# the phase is a property of the runtime, not of the call site, and `reset!` puts it back.
function (c::ProgramRuntime{names, K})(; tick::Bool = true) where {names, K}
    k = (c.counter[] += 1)
    ticks = map(d -> tick && (k - 1) % d == 0, c.divisors)
    out = SynchJulia.step!(c.exe, ticks..., c.gains, c.auto)
    return NamedTuple{names}(values(out))
end

"Resets the program's own state, and the I/O counters and loop timing; the device and the log stay as they are."
function SynchToolkit.reset!(c::ProgramRuntime)
    SynchToolkit.reset!(c.exe)
    c.counter[] = 0
    reset_hardware_counters!()
    return c
end

log_file(c::ProgramRuntime) = c.log.file

# ---------------------------------------------------------------------------
## Running it in this process
# ---------------------------------------------------------------------------
"""
    with_rig(body; mode, arm_deg, card_options, log, traj=nothing, disable_gc=true, prepare=nothing)

Open the device and the log around `body()` and close both again whatever happens, returning
what `body` returned.

Every way of putting a program on the rig in this process needs the same sequence -- open the
device, reset its counters and timing, open the log (and the trajectory a replay reads), run
with the garbage collector off, and close the device and the log even on an exception, since
the program cannot unwind a motor write it has already made. [`run_inprocess!`](@ref) ticks a
compiled node inside it and [`run_ode!`](@ref) lets an ODE solver step a model.

`prepare` runs after the device is open but before the log is opened and the collector is
switched off: it is for whatever must happen with the device in hand but outside the timed,
logged run -- resetting the program's own state, or solving the problem once so that
everything the run touches is already compiled. The counters and the timing are reset after
it, so the run starts its own timeline whatever `prepare` did to the device.

The collector is off for the duration so that no collection pause lands inside a period; the
programs here allocate next to nothing per tick. The MPC is the exception -- its acados
callbacks allocate a few MB per tick -- so long runs of it pass `disable_gc = false` and
accept an occasional overrun instead.
"""
function with_rig(body; mode::Symbol, arm_deg, card_options, log::ProgramLog,
                  traj::Union{Nothing, ProgramTrajectory} = nothing,
                  disable_gc::Bool = true, prepare = nothing)
    open_hardware!(mode; arm_deg, card_options)
    try
        prepare === nothing || prepare()
        reset_hardware_counters!()
        open_log!(log)
        open_traj!(traj)          # no-op when the program replays nothing
        GC.gc()
        disable_gc && GC.enable(false)
        return body()
    finally
        GC.enable(true)
        close_hardware!()
        close_log!()
    end
end

"""
    run_inprocess!(ctrl::ProgramRuntime; Tf, arm_deg=0.0, card_options=nothing, mode=:hil, disable_gc=true)

Tick `ctrl` against the device every `Ts` for `Tf` seconds from a timing loop in this process,
and return `(; rows, ticks, log_file, timing)`.

This opens the device and the log, runs the loop, and closes both. It does no logging of its
own -- the program writes the file -- so the loop body is a single call, and `rows` coming
back equal to `ticks` is the check that the program wrote one row per tick. `timing` is
`(; median_dt, max_dt)` of the achieved period, for confirming the loop kept up. For a
multirate program `Ts` is the fastest clock's period and `rows` counts the logger's clock.

`card_options` overrides the driver's card options (`nothing` keeps `qube_hw.c`'s own
default, `""` leaves the driver on its own). `mode = :callback` runs against whatever
[`bind_hardware!`](@ref) installed, which is how this is exercised without a rig.

The device and the log are opened and closed by [`with_rig`](@ref), which also switches the
garbage collector off for the duration -- see there for `disable_gc`.
"""
function run_inprocess!(ctrl::ProgramRuntime; Tf, arm_deg = 0.0,
                        card_options::Union{Nothing, AbstractString} = nothing,
                        mode::Symbol = :hil, disable_gc::Bool = true)
    Ts = ctrl.Ts
    N = round(Int, Tf / Ts)
    # The loop's own variables stay inside the closure and come back as its return value
    # rather than being assigned into locals of this frame: writing to a captured local from
    # inside a closure boxes it, and an untyped box is an allocation per tick in the one
    # place there is no collector to take it back.
    n, tstamp = with_rig(; mode, arm_deg, card_options, log = ctrl.log, traj = ctrl.traj,
                          disable_gc, prepare = () -> SynchToolkit.reset!(ctrl)) do
        tstamp = Vector{Float64}(undef, N)     # preallocated: GC is disabled here
        n = 0
        try
            t0 = time()
            t_next = t0 + Ts
            for i in 1:N
                ctrl()
                tstamp[i] = time() - t0
                n = i
                # Absolute schedule rather than "sleep Ts": a step that overruns is absorbed by
                # the next period instead of shifting every period after it. `systemsleep`
                # because `sleep`'s resolution is of the same order as Ts itself.
                dt = t_next - time()
                dt > 0 && Libc.systemsleep(dt)
                t_next += Ts
            end
        catch e
            # Report what the run managed rather than losing it: the device and the log are
            # closed by `with_rig` either way.
            @error "Terminating run" e
        end
        return n, tstamp
    end
    st = log_state()
    st.error && @warn "the log was closed early by a write error" file = st.filename
    dts = n > 1 ? diff(view(tstamp, 1:n)) : Float64[]
    return (; rows = st.rows, ticks = n, log_file = ctrl.log.file,
              timing = (; median_dt = _median(dts),
                          max_dt = isempty(dts) ? NaN : maximum(dts)))
end

# ---------------------------------------------------------------------------
## Running the model as an ODE, paced by itself
# ---------------------------------------------------------------------------
"""
    run_ode!(ctor, prob; log_file=nothing, arm_deg=0.0, card_options=nothing, mode=:hil, warmup=nothing, disable_gc=true) -> sol

Solve `prob`, an `ODEProblem` of a program's model, against the device, and return the solution.
For the MPC programs the solution is what `MPCComponents.mpc_gui(model, sol)` takes.

[`run_inprocess!`](@ref) ticks a node that keeps nothing but its outputs. Here nothing is compiled
to a node: an ODE solver steps the clocked partitions of the model, with the hardware I/O
happening inside the ticks exactly as in the program, so that every clocked variable ends up in
the solution -- for the MPC, the predicted trajectories and solver residuals. The caller builds
the model and the problem, so that both can be kept between runs:

```julia
spec = program_spec(FurutaMPCHardware)
model = FurutaMPCHardware(; name = spec.name, Ts, Np, log_file, spec.ode_kwargs...)
ssys = mtkcompile(model; additional_passes = [SynchToolkit.compile_lustre])
prob = ODEProblem(ssys, Pair[], (0.0, Tf); build_initializeprob = false)
sol = run_ode!(FurutaMPCHardware, prob; log_file)
```

The model has to be built with the spec's `ode_kwargs`: `realtime = true` makes
`HardwareDiagnostics` pace the ticks on the wall clock, without which the solver steps the
clocked partition as fast as it can against the device, and for the MPC `output_trajectories =
true` makes it record its predictions. The model is purely discrete, and ModelingToolkit's
initialization problem cannot be built for its array-valued clocked variables, hence
`build_initializeprob = false`. `log_file` must be the file the model was built with: the
model's `DataLogger` writes it, and this opens it with the program's columns.

Warm, a tick costs what the program's does; cold, the first MPC solve compiles for tens of
seconds, which would leave every tick of a paced run behind schedule. So a problem over the same
system is first solved for `warmup` seconds with the spec's `ode_warmup` overrides applied (for
the MPC, `command_umax = 0`: the encoders are read, nothing is written to the motor), the timing
is reset, and only then is `prob` solved and logged. `warmup` defaults to a fraction of a second
when the spec can clamp the command and to 0 otherwise. Opening and closing the device and the
log around all of that, and switching the garbage collector off for the run, is
[`with_rig`](@ref)'s job -- see there for `disable_gc`, which is worth setting to `false` for a
long MPC run. `mode = :callback` runs against whatever [`bind_hardware!`](@ref) installed.

For a multirate model `realtime = true` paces the slowest clock only, so the fast reads of each
period bunch just before its slot instead of spreading over it -- fine for inspecting the MPC's
predictions, which is what this route is for, and not representative of the compiled program's
timing.
"""
function run_ode!(ctor, prob; log_file = nothing, arm_deg = 0.0,
                  card_options::Union{Nothing, AbstractString} = nothing,
                  mode::Symbol = :hil, warmup = nothing, disable_gc::Bool = true)
    spec = program_spec(ctor)
    ensure_qube_hw()
    ensure_qube_log()
    log = log_file === nothing ? spec.log() : spec.log(log_file)
    ssys = prob.f.sys
    warmup = something(warmup, spec.ode_warmup === nothing ? 0.0 : 0.2)
    prepare = nothing
    if warmup > 0
        spec.ode_warmup === nothing &&
            throw(ArgumentError("the program $(spec.name) has no `ode_warmup` overrides that \
                                 keep it from writing to the motor, so it cannot be warmed up \
                                 against the device; pass `warmup = 0`"))
        # Compile everything the run touches before the log is open, so the warm-up's ticks
        # are neither logged nor timed. `with_rig` resets the counters and the timing after
        # it, so the run that follows starts from zero.
        prepare = function ()
            wp = ODEProblem(ssys, spec.ode_warmup(ssys), (0.0, Float64(warmup));
                            build_initializeprob = false)
            solve(wp)
            return nothing
        end
    end
    return with_rig(; mode, arm_deg, card_options, log, disable_gc, prepare) do
        solve(prob)
    end
end

# ---------------------------------------------------------------------------
## Reading a log back
# ---------------------------------------------------------------------------
"""
    read_log(path) -> NamedTuple of vectors

Parse a log written by a `DataLogger` inside a program: a tab-separated header line naming
the columns, then one numeric row per tick. Returns a Tables.jl-compatible column table
keyed by the header's names, so it follows whatever columns the program logged rather than a
second copy of the layout kept here.

A row cut short by the run being interrupted is dropped rather than erroring -- the log is
flushed on close, so the last line may be partial.
"""
function read_log(path)
    lines = readlines(path)
    isempty(lines) && throw(ArgumentError("$path is empty"))
    cols = Symbol.(split(strip(lines[1]), '\t'))
    rows = Vector{Vector{Float64}}()
    for l in Iterators.drop(lines, 1)
        isempty(strip(l)) && continue
        vals = tryparse.(Float64, split(strip(l), '\t'))
        (length(vals) == length(cols) && all(!isnothing, vals)) || continue
        push!(rows, Vector{Float64}(vals))
    end
    data = isempty(rows) ? [Float64[] for _ in cols] :
           [[r[j] for r in rows] for j in eachindex(cols)]
    return NamedTuple{Tuple(cols)}(Tuple(data))
end

# Timing of a run recovered from its log, for the targets that do not tick the loop here:
# the program logs the period it achieved, so a fetched log is as good as a local measurement.
function log_timing(path)
    isfile(path) || return (; median_dt = NaN, max_dt = NaN)
    d = try
        read_log(path)
    catch
        return (; median_dt = NaN, max_dt = NaN)
    end
    haskey(d, :dt) && length(d.dt) > 1 || return (; median_dt = NaN, max_dt = NaN)
    # The first row's `dt` has no predecessor and is logged as 0.
    dts = d.dt[2:end]
    return (; median_dt = _median(dts), max_dt = maximum(dts))
end

# Statistics is not a dependency of this package and one median does not justify making it
# one; the achieved periods are a short vector, so sorting a copy costs nothing.
function _median(v)
    isempty(v) && return NaN
    s = sort(collect(v))
    n = length(s)
    return isodd(n) ? s[(n + 1) ÷ 2] : (s[n ÷ 2] + s[n ÷ 2 + 1]) / 2
end
