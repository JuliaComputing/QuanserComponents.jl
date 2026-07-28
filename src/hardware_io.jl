# Hardware I/O from inside the synchronous controller, on every target.
#
# `HardwareMeasurement` and `HardwareCommand` (dyad/hardware_loop.dyad) do the
# encoder read and the amplifier write *inside* the compiled node rather than in
# a surrounding control loop. The implementation is a small C library,
# csrc/qube_hw.c, and the operators below are one-line `ccall`s into it.
#
# Using named `ccall((:sym, lib), ...)` rather than Julia callbacks is what makes
# one controller definition serve all three targets:
#
#   backend = :julia   Julia ccalls the shared library directly.
#   backend = :c       SynchCompiler links the library into the node's .so.
#   export_c           the emitted top.c contains
#                        extern double qube_hw_measure(double arg1);
#                      plus a named call site, so the standalone C links against
#                      csrc/qube_hw.c with no Julia involved.
#
# The pointer form (`@cfunction`, `Ptr` arguments, FunctionWrappers) would work
# on the Julia backend but `export_c` rejects it outright -- a process-local
# function pointer is meaningless in a separately built C module. Hence the
# fixed symbol names and the `const` library path, both of which the C backend
# needs statically.
#
# Two properties are load-bearing and are asserted by the tests:
#
#  1. **Exactly one read and one write per tick.** `stkcompile` does no CSE, no
#     DCE and no observed-variable inlining, so one equation is one call.
#  2. **Ordering.** Equation scheduling only respects data dependencies, so the
#     two cache accessors take a `dep` argument that carries no information other
#     than "the read has happened". Without it they could be scheduled before the
#     read and serve the previous tick's values. (Same trick as
#     `DiscreteComponents`' `seeded_rand(seed, td)`.)

using ModelingToolkit

export build_qube_hw!, open_hardware!, close_hardware!, bind_hardware!,
       hardware_counters, hardware_state, quanser_sdk_flags, hardware_card_options

# The `ccall` library must be a compile-time constant for the C backend to link
# it statically, so the path is fixed and `build_qube_hw!` writes to it.
const QUBE_HW_SRC = normpath(joinpath(@__DIR__, "..", "csrc", "qube_hw.c"))
const QUBE_HW_DIR = normpath(joinpath(@__DIR__, "..", "deps"))
const QUBE_HW_LIB = joinpath(QUBE_HW_DIR,
    "libqube_hw." * (Sys.iswindows() ? "dll" : Sys.isapple() ? "dylib" : "so"))

const QUANSER_HIL_DIR = "/opt/quanser/hil_sdk"

# Libraries the HIL API needs, the same for both SDK layouts below.
const QUANSER_LIBS = ["-lhil", "-lquanser_runtime", "-lquanser_common",
                      "-lrt", "-lpthread", "-ldl", "-lm"]

"""
    quanser_sdk_flags(; quanser_dir=QUANSER_HIL_DIR)

Locate the Quanser HIL SDK and return `(; found, layout, cflags, ldflags)`.

Two layouts are in the wild and `csrc/qube_hw.c` compiles against both unchanged —
only the search paths differ:

  `:sdk_dir`  the older HIL SDK, headers under `\$quanser_dir/include` and static
              libraries under `\$quanser_dir/lib`.
  `:system`   the current `quanser-sdk` Debian packages, headers under
              `/usr/include/quanser` and shared libraries on the default path
              (this is what the Raspberry Pi target uses).

`:sdk_dir` wins if both are present, since an explicitly installed SDK directory is
the more deliberate choice.
"""
function quanser_sdk_flags(; quanser_dir::AbstractString = QUANSER_HIL_DIR)
    if isfile(joinpath(quanser_dir, "include", "hil.h"))
        return (; found = true, layout = :sdk_dir,
                  cflags = ["-I$(joinpath(quanser_dir, "include"))"],
                  ldflags = ["-L$(joinpath(quanser_dir, "lib"))"])
    elseif isfile("/usr/include/quanser/hil.h")
        return (; found = true, layout = :system,
                  cflags = ["-I/usr/include/quanser"], ldflags = String[])
    else
        return (; found = false, layout = :none, cflags = String[], ldflags = String[])
    end
end

"Path of a C compiler to build the shim with, or `nothing`."
function _c_compiler()
    for c in ("cc", "gcc", "clang")
        p = Sys.which(c)
        p === nothing || return p
    end
    # SynchCompiler ships clang through a Clang_unified_jll extension; use it if
    # the extension is loaded (it is whenever the `:c` backend is available).
    try
        return SynchCompiler.cc()
    catch
        return nothing
    end
end

"""
    build_qube_hw!(; hil=quanser_sdk_flags().found, force=false, quanser_dir=QUANSER_HIL_DIR)

Compile `csrc/qube_hw.c` into `deps/libqube_hw.\$(Libdl.dlext)`, the library the
generated controller calls into. Called automatically before the controller is
compiled; call it directly only to change `hil` or to force a rebuild.

With `hil=true` the Quanser HIL SDK is linked in and `open_hardware!(:hil)`
talks to the board; with `hil=false` only `open_hardware!(:callback)` works,
which is enough to run the controller against a simulator. The default follows
whether the SDK is installed.

Returns the library path.
"""
function build_qube_hw!(; hil::Bool = quanser_sdk_flags().found, force::Bool = false,
                          quanser_dir::AbstractString = QUANSER_HIL_DIR)
    mkpath(QUBE_HW_DIR)
    stamp = QUBE_HW_LIB * ".stamp"
    want = string(hil, "\n", quanser_sdk_flags(; quanser_dir).layout, "\n", mtime(QUBE_HW_SRC))
    if !force && isfile(QUBE_HW_LIB) && isfile(stamp) && read(stamp, String) == want
        return QUBE_HW_LIB
    end
    cc = _c_compiler()
    cc === nothing && error("""
        No C compiler found (tried cc, gcc, clang and SynchCompiler's Clang_unified_jll).
        One is needed to build $QUBE_HW_SRC, the hardware I/O the generated
        controller calls into.""")
    sdk = quanser_sdk_flags(; quanser_dir)
    hil && !sdk.found && error("""
        hil=true but no Quanser HIL SDK was found: neither $(joinpath(quanser_dir, "include", "hil.h"))
        nor /usr/include/quanser/hil.h exists.""")
    flags = ["-O2", "-Wall", "-fPIC", "-shared"]
    hil && append!(flags, ["-DQUBE_HW_HAVE_HIL"], sdk.cflags)
    libs = hil ? vcat(sdk.ldflags, QUANSER_LIBS) : String[]
    run(`$cc $flags -o $QUBE_HW_LIB $QUBE_HW_SRC $libs`)
    write(stamp, want)
    return QUBE_HW_LIB
end

# Build lazily rather than in `__init__`: importing the package should not
# require a C compiler, only compiling a controller should.
_qube_hw_ready = false
function ensure_qube_hw()
    global _qube_hw_ready
    _qube_hw_ready && return QUBE_HW_LIB
    build_qube_hw!()
    _qube_hw_ready = true
    return QUBE_HW_LIB
end

# ---------------------------------------------------------------------------
## The operators the controller node calls
# ---------------------------------------------------------------------------
@register_symbolic hw_measure(dep::Real)::Real
@register_symbolic hw_shoulder(dep::Real)::Real
@register_symbolic hw_elbow(dep::Real)::Real
@register_symbolic hw_write(u::Real, umax::Real)::Real

"""
    hw_measure(dep) -> shoulder

The one effectful read: samples both encoders and caches them. The returned
shoulder angle doubles as the dependency token that orders [`hw_shoulder`](@ref)
and [`hw_elbow`](@ref) after this call. `dep` is ignored.
"""
hw_measure(dep::Real)::Float64 =
    ccall((:qube_hw_measure, QUBE_HW_LIB), Cdouble, (Cdouble,), dep)

"Cached shoulder angle. `dep` only forces this to be scheduled after `hw_measure`."
hw_shoulder(dep::Real)::Float64 =
    ccall((:qube_hw_shoulder, QUBE_HW_LIB), Cdouble, (Cdouble,), dep)

"Cached elbow angle. `dep` only forces this to be scheduled after `hw_measure`."
hw_elbow(dep::Real)::Float64 =
    ccall((:qube_hw_elbow, QUBE_HW_LIB), Cdouble, (Cdouble,), dep)

"""
    hw_write(u, umax) -> u_applied

Clamp `u` to `[-umax, umax]`, write it to the amplifier and return what was
applied. Ordered after the read because `u` depends on the measured angles.
"""
hw_write(u::Real, umax::Real)::Float64 =
    ccall((:qube_hw_write, QUBE_HW_LIB), Cdouble, (Cdouble, Cdouble), u, umax)

# ---------------------------------------------------------------------------
## Driver-side API
# ---------------------------------------------------------------------------
const QUBE_HW_MODE = (callback = Cint(0), hil = Cint(1))

# Callback trampolines. These must be plain top-level functions so `@cfunction`
# yields a static pointer; the Julia closures they forward to live in these Refs.
const MEASURE_CB = Ref{Any}(() -> (0.0, 0.0))
const CONTROL_CB = Ref{Any}(u -> nothing)

function _measure_trampoline(shoulder::Ptr{Cdouble}, elbow::Ptr{Cdouble})::Cvoid
    y = MEASURE_CB[]()
    unsafe_store!(shoulder, Float64(y[1]))
    unsafe_store!(elbow, Float64(y[2]))
    return nothing
end

function _control_trampoline(u::Cdouble)::Cvoid
    CONTROL_CB[](u)
    return nothing
end

"""
    open_hardware!(mode = :hil; arm_deg = 0.0)

Open the hardware I/O in `:hil` mode (talk to the QUBE through the HIL SDK) or
`:callback` mode (call the handlers installed by [`bind_hardware!`](@ref)).
Clears the cached angles and the call counters.

In `:hil` mode this enables the amplifier, zeroes the motor and records the
current encoder counts as homing offsets. `arm_deg` says where the arm physically
is at that moment, in degrees, and is added to every shoulder reading — so you do
not have to move the arm to its home position first: park it roughly centred, pass
how far off it is, and 0 still means centred. This is the same calibration
`QuanserInterface.home!(process, arm_deg)` performs. The pendulum must hang
straight down; that reading is always zeroed.

`arm_deg` is ignored in `:callback` mode, where the handler is expected to return
angles that are already calibrated.

`card_options` overrides the card-specific options string applied after `hil_open`
(`nothing` keeps the built-in default, `""` leaves the driver on its own defaults).
This is where `deadband_compensation` lives, which is part of the command-to-torque
path — see `csrc/qube_hw.h`. Query the value in force with [`hardware_card_options`](@ref).
"""
function open_hardware!(mode::Symbol = :hil; arm_deg = 0.0,
                        card_options::Union{Nothing, AbstractString} = nothing)
    ensure_qube_hw()
    card_options === nothing ||
        ccall((:qube_hw_set_card_options, QUBE_HW_LIB), Cvoid, (Cstring,), card_options)
    m = get(QUBE_HW_MODE, mode) do
        throw(ArgumentError("unknown hardware mode $(repr(mode)); use :hil or :callback"))
    end
    r = ccall((:qube_hw_open, QUBE_HW_LIB), Cint, (Cint, Cdouble), m, deg2rad(arm_deg))
    r == 0 || error("qube_hw_open($mode) failed with code $r" *
                    (mode === :hil && !have_hil() ?
                     "; the library was built without HIL support, rebuild with " *
                     "build_qube_hw!(; hil = true, force = true)" : ""))
    return nothing
end

"Zero the motor and release the device. Safe to call when nothing is open."
function close_hardware!()
    # Called from `finally` blocks, so tolerate never having built the library.
    ensure_qube_hw()
    ccall((:qube_hw_close, QUBE_HW_LIB), Cvoid, ())
end

"""
    hardware_card_options() -> String

The card-specific options string that `open_hardware!(:hil)` will apply (or applied).
Empty means the driver is left on its own defaults.
"""
hardware_card_options() =
    (ensure_qube_hw(); unsafe_string(ccall((:qube_hw_card_options, QUBE_HW_LIB), Cstring, ())))

"True if `deps/libqube_hw` was built with Quanser HIL SDK support."
have_hil() = (ensure_qube_hw(); ccall((:qube_hw_have_hil, QUBE_HW_LIB), Cint, ()) != 0)

"""
    bind_hardware!(; measure, control)

Drive the controller from Julia instead of the board: install `measure`, a
zero-argument function returning the two angles `(shoulder, elbow)` in radians,
and `control`, called with the clamped voltage. Opens `:callback` mode.

This is how the controller is exercised against a simulator, e.g.

```julia
process = QuanserInterface.QubeServoPendulumSimulator(; Ts)
bind_hardware!(measure = () -> QuanserInterface.measure(process),
               control = u -> QuanserInterface.control(process, [u]))
```

On the real rig use [`open_hardware!`](@ref)`(:hil)` instead, which needs no
Julia in the loop at all and is the same code path the exported C takes.
"""
function bind_hardware!(; measure, control)
    ensure_qube_hw()
    MEASURE_CB[] = measure
    CONTROL_CB[] = control
    ccall((:qube_hw_set_callbacks, QUBE_HW_LIB), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}),
          @cfunction(_measure_trampoline, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble})),
          @cfunction(_control_trampoline, Cvoid, (Cdouble,)))
    open_hardware!(:callback)
    return nothing
end

"""
    hardware_counters() -> (; n_measure, n_write)

Reads and writes performed since the device was opened. The tests assert one of
each per tick.
"""
hardware_counters() = (
    n_measure = Int(ccall((:qube_hw_n_measure, QUBE_HW_LIB), Clong, ())),
    n_write = Int(ccall((:qube_hw_n_write, QUBE_HW_LIB), Clong, ())),
)

"Reset the call counters without reopening the device."
reset_hardware_counters!() = ccall((:qube_hw_reset_counters, QUBE_HW_LIB), Cvoid, ())

"""
    hardware_state() -> (; shoulder, elbow, u, count_shoulder, count_elbow)

Most recent measured angles and applied voltage, plus the raw encoder counts (HIL
mode only) for diagnosing counter glitches. The controller returns the same
angles and voltage as outputs; this is for inspecting them out of band.
"""
hardware_state() = (
    shoulder = ccall((:qube_hw_last_shoulder, QUBE_HW_LIB), Cdouble, ()),
    elbow = ccall((:qube_hw_last_elbow, QUBE_HW_LIB), Cdouble, ()),
    u = ccall((:qube_hw_last_u, QUBE_HW_LIB), Cdouble, ()),
    count_shoulder = Int(ccall((:qube_hw_last_count_shoulder, QUBE_HW_LIB), Clong, ())),
    count_elbow = Int(ccall((:qube_hw_last_count_elbow, QUBE_HW_LIB), Clong, ())),
)
