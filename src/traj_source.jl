# Feeding a recorded input sequence into the synchronous program, on every target.
#
# The mirror image of data_log.jl. The `TrajectorySource` component (dyad/identification.dyad)
# reads one sample per tick from *inside* the compiled node, exactly as `DataLogger` writes one
# row per tick from inside it, and for the same reason: an open-loop replay is then the program's
# own doing rather than something a surrounding loop feeds it, so the Julia backend, the
# in-process `:c` backend and `export_c` all replay the same file the same way.
#
# The implementation is csrc/qube_traj.c and the single operator below is a one-line `ccall`
# into it -- see the header comment in src/hardware_io.jl for why the named-`ccall` form is the
# one that survives all three targets.
#
# The file name cannot cross the node's interface (every signal there is a `double`), so the
# component owns it as a **structural** parameter and the driver opens the file:
# [`open_traj!`](@ref) from Julia, `qube_traj_open` from the exported C harness, both with the
# value the model was built with.

using ModelingToolkit

export open_traj!, close_traj!, traj_state, build_qube_traj!

# Fixed path for the same reason as `QUBE_HW_LIB`: the `:c` backend links the library named in
# the `ccall` statically, so it must be a compile-time constant.
const QUBE_TRAJ_SRC = normpath(joinpath(@__DIR__, "..", "csrc", "qube_traj.c"))
const QUBE_TRAJ_LIB = joinpath(QUBE_HW_DIR,
    "libqube_traj." * (Sys.iswindows() ? "dll" : Sys.isapple() ? "dylib" : "so"))

"""
    build_qube_traj!(; force=false)

Compile `csrc/qube_traj.c` into `deps/libqube_traj.\$(dlext)`, the library the
`TrajectorySource` component calls into. Called automatically before a model containing one is
compiled. Needs nothing but a C compiler and libc.

Returns the library path.
"""
function build_qube_traj!(; force::Bool = false)
    mkpath(QUBE_HW_DIR)
    stamp = QUBE_TRAJ_LIB * ".stamp"
    want = string(mtime(QUBE_TRAJ_SRC))
    if !force && isfile(QUBE_TRAJ_LIB) && isfile(stamp) && read(stamp, String) == want
        return QUBE_TRAJ_LIB
    end
    cc = _c_compiler()
    cc === nothing && error("""
        No C compiler found (tried cc, gcc, clang and SynchCompiler's Clang_unified_jll).
        One is needed to build $QUBE_TRAJ_SRC, the trajectory playback the generated
        program calls into.""")
    run(`$cc -O2 -Wall -fPIC -shared -o $QUBE_TRAJ_LIB $QUBE_TRAJ_SRC`)
    write(stamp, want)
    return QUBE_TRAJ_LIB
end

_qube_traj_ready = false
function ensure_qube_traj()
    global _qube_traj_ready
    _qube_traj_ready && return QUBE_TRAJ_LIB
    build_qube_traj!()
    _qube_traj_ready = true
    return QUBE_TRAJ_LIB
end

# ---------------------------------------------------------------------------
## The operator the program calls
# ---------------------------------------------------------------------------
@register_symbolic traj_value(index::Real)::Real

"""
    traj_value(index) -> u

The `index`-th sample (1-based) of the open trajectory, or `0.0` when none is open or the
index is past the end — a replay that outlives its trajectory stops driving.

Needs no dependency token, unlike the hardware accessors: the index *is* the dependency, and
it comes from the program's own tick counter.
"""
traj_value(index::Real)::Float64 =
    ccall((:qube_traj_value, QUBE_TRAJ_LIB), Cdouble, (Cdouble,), index)

# ---------------------------------------------------------------------------
## Driver-side API
# ---------------------------------------------------------------------------
"""
    open_traj!(filename; column=2) -> nsamples

Load `column` (1-based) of `filename` for the `TrajectorySource` to replay, and return how many
samples were read. Whitespace- or tab-separated text, one sample per line; a header line is
skipped, being unparseable as a number.

Pass the settings the model's `TrajectorySource` was built with rather than a second copy of
them — [`IdentificationController`](@ref) and the analysis both do.
"""
function open_traj!(filename::AbstractString; column::Integer = 2)
    ensure_qube_traj()
    r = ccall((:qube_traj_open, QUBE_TRAJ_LIB), Cint, (Cstring, Cint), filename, column)
    r == 0 || error("open_traj!($(repr(filename)), column = $column) failed with code $r" *
                    (r == -2 ? " (could not open the file for reading)" :
                     r == -4 ? " (no numeric rows found — is the column right?)" : ""))
    return traj_state().n
end

"Release the loaded trajectory. Safe to call when none is open."
close_traj!() = (ensure_qube_traj(); ccall((:qube_traj_close, QUBE_TRAJ_LIB), Cvoid, ()))

"""
    traj_state() -> (; n, filename, error)

Samples loaded, the file they came from, and whether the program has read past the end (which
returns 0 rather than faulting, so this is the only place that shows up).
"""
traj_state() = (ensure_qube_traj(); (
    n = Int(ccall((:qube_traj_length, QUBE_TRAJ_LIB), Clong, ())),
    filename = unsafe_string(ccall((:qube_traj_filename, QUBE_TRAJ_LIB), Cstring, ())),
    error = ccall((:qube_traj_error, QUBE_TRAJ_LIB), Cint, ()) != 0,
))
