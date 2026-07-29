# Data logging from inside the synchronous program, on every target.
#
# The `DataLogger` component (dyad/friction.dyad) writes one row per tick from
# *inside* the compiled node instead of from a surrounding loop, exactly as
# `HardwareMeasurement`/`HardwareCommand` do their I/O there. The implementation is
# csrc/qube_log.c and the single operator below is a one-line `ccall` into it, so
# the same component definition serves the Julia backend, the in-process `:c`
# backend and `export_c` — see the header comment in src/hardware_io.jl for why the
# named-`ccall` form is the one that survives all three.
#
# What the program cannot carry is the *filename*: every signal crossing a
# synchronous node's interface is a `double`. So the component owns the filename as
# a **structural** parameter — a build-time Julia value that never enters the
# equations — and the driver opens the file. That stays one source of truth because
# the generator hands the same value to the model and to `open_log!`
# (`generate_friction_controller` in src/friction.jl), and the column names and
# count are shared consts in dyad/definitions.jl.
#
# With no log open `log_row` is a no-op returning 0, so a model containing a
# `DataLogger` also runs when nothing wants the data (the simulator tests rely on
# this).

using ModelingToolkit

export open_log!, close_log!, flush_log!, log_state, build_qube_log!

# Fixed path for the same reason as `QUBE_HW_LIB`: the `:c` backend links the
# library named in the `ccall` statically, so it must be a compile-time constant.
const QUBE_LOG_SRC = normpath(joinpath(@__DIR__, "..", "csrc", "qube_log.c"))
const QUBE_LOG_LIB = joinpath(QUBE_HW_DIR,
    "libqube_log." * (Sys.iswindows() ? "dll" : Sys.isapple() ? "dylib" : "so"))

# Columns `log_row` accepts. Must equal QUBE_LOG_MAX_COLS in csrc/qube_log.h and the
# number of inputs on the `DataLogger` component.
const QUBE_LOG_MAX_COLS = 8

"""
    build_qube_log!(; force=false)

Compile `csrc/qube_log.c` into `deps/libqube_log.\$(dlext)`, the library the
`DataLogger` component calls into. Called automatically before a model containing
one is compiled. Needs nothing but a C compiler and libc.

Returns the library path.
"""
function build_qube_log!(; force::Bool = false)
    mkpath(QUBE_HW_DIR)
    stamp = QUBE_LOG_LIB * ".stamp"
    want = string(mtime(QUBE_LOG_SRC))
    if !force && isfile(QUBE_LOG_LIB) && isfile(stamp) && read(stamp, String) == want
        return QUBE_LOG_LIB
    end
    cc = _c_compiler()
    cc === nothing && error("""
        No C compiler found (tried cc, gcc, clang and SynchCompiler's Clang_unified_jll).
        One is needed to build $QUBE_LOG_SRC, the data logging the generated
        program calls into.""")
    run(`$cc -O2 -Wall -fPIC -shared -o $QUBE_LOG_LIB $QUBE_LOG_SRC`)
    write(stamp, want)
    return QUBE_LOG_LIB
end

_qube_log_ready = false
function ensure_qube_log()
    global _qube_log_ready
    _qube_log_ready && return QUBE_LOG_LIB
    build_qube_log!()
    _qube_log_ready = true
    return QUBE_LOG_LIB
end

# ---------------------------------------------------------------------------
## The operator the program calls
# ---------------------------------------------------------------------------
# One operator of fixed arity rather than one per column: a registered operator has a
# fixed signature — there is no variable-argument form, and an array argument would put
# an array-valued signal inside the clocked partition. A single call per tick also keeps
# the "one row per tick" property as easy to assert as the hardware I/O's
# one-read-one-write. Arguments past the `ncols` given to `open_log!` are accepted and
# discarded, so the component's unused inputs can be tied off to zero.
@register_symbolic log_row(u1::Real, u2::Real, u3::Real, u4::Real,
                           u5::Real, u6::Real, u7::Real, u8::Real)::Real

"""
    log_row(u1, ..., u8) -> row

Append one tab-separated row of the first `ncols` arguments to the open log and
return the 1-based number of the row just written; `0` when no log is open.

Ordering needs no dependency token, unlike the hardware accessors: the logged
values are what the program computed this tick, so the data dependency is real.
"""
log_row(u1::Real, u2::Real, u3::Real, u4::Real,
        u5::Real, u6::Real, u7::Real, u8::Real)::Float64 =
    ccall((:qube_log_row, QUBE_LOG_LIB), Cdouble,
          (Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble),
          u1, u2, u3, u4, u5, u6, u7, u8)

# ---------------------------------------------------------------------------
## Driver-side API
# ---------------------------------------------------------------------------
"""
    open_log!(filename; header="", ncols=QUBE_LOG_MAX_COLS)

Open `filename` for the `DataLogger` to write to, truncating any existing file, and
reset the row counter. `header` is written verbatim as the first line when
non-empty, so it should already be tab-separated; `ncols` says how many of
`log_row`'s arguments to write.

Pass the settings the model's `DataLogger` was built with rather than a second copy
of them; for the friction experiment those are the `FRICTION_LOG_*` consts in
dyad/definitions.jl, which the Dyad component and the generator both reference.
"""
function open_log!(filename::AbstractString; header::AbstractString = "",
                   ncols::Integer = QUBE_LOG_MAX_COLS)
    ensure_qube_log()
    r = ccall((:qube_log_open, QUBE_LOG_LIB), Cint, (Cstring, Cstring, Cint),
              filename, header, ncols)
    r == 0 || error("open_log!($(repr(filename))) failed with code $r" *
                    (r == -2 ? " (could not open the file for writing)" : ""))
    return nothing
end

"Flush buffered rows so a viewer tailing the file sees the run in progress."
flush_log!() = (ensure_qube_log(); ccall((:qube_log_flush, QUBE_LOG_LIB), Cvoid, ()))

"Flush and close the log. Safe to call when none is open."
close_log!() = (ensure_qube_log(); ccall((:qube_log_close, QUBE_LOG_LIB), Cvoid, ()))

"""
    log_state() -> (; rows, cols, filename, error)

Rows written since [`open_log!`](@ref), columns being written, the file being
written, and whether a write has failed (which closes the log — the program keeps
running, so this is the only place a lost log shows up).
"""
log_state() = (ensure_qube_log(); (
    rows = Int(ccall((:qube_log_rows, QUBE_LOG_LIB), Clong, ())),
    cols = Int(ccall((:qube_log_cols, QUBE_LOG_LIB), Cint, ())),
    filename = unsafe_string(ccall((:qube_log_filename, QUBE_LOG_LIB), Cstring, ())),
    error = ccall((:qube_log_error, QUBE_LOG_LIB), Cint, ()) != 0,
))
