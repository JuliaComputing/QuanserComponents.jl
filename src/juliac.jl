# Deploying a program as a statically compiled Julia binary.
#
# The third target, next to running in this process and exporting C: JuliaC compiles the
# program into a standalone executable with no Julia installation behind it. The layout
# mirrors harness.jl's C export closely, because the deployed thing is the same -- the node
# does its own I/O and its own logging, so what surrounds it is only timing:
#
#   export_program_c        -> top.c + csrc + run_hardware.c   -> `make`  -> ./run_hardware
#   export_program_juliac   -> an app package (node as source) -> JuliaC  -> bin/<app>
#
# What makes trimming work is SynchJulia's static-compilation pattern
# (JuliaComputing/SynchJulia.jl#203): the emitted package defines the node and constructs its
# `SynchExecutable` at package top level, i.e. during *precompilation*, so both are serialized
# into its package image; `@main` then only ticks that executable. Two further conditions,
# both settled upstream, are what let `step!`/`reset!` themselves be statically resolved:
# SynchJulia >= 0.4.3 (JuliaComputing/SynchJulia.jl#226) and its `dynamic_execution = false`
# preference, which the emitted Project.toml sets.
#
# The emitted package depends on nothing from this one. It carries its own copies of the
# `csrc/` implementations the node calls into, builds them next to itself, and defines the
# operator wrappers over them -- see `emit_program_ffi` and `compile_program_source`.

using UUIDs: UUID, uuid5

export export_program_juliac, build_program_juliac, run_program_juliac, juliac_available

# Versions the emitted app is pinned against. SynchJulia 0.4 is the first release whose
# executables survive serialization into a package image, and 0.4.3 the first whose `step!`
# converts a heterogeneous input tuple without dynamic dispatch, which `--trim=safe` requires.
# JuliaC's `--trim` needs Julia 1.13.
const APP_COMPAT = (SynchJulia = "0.4.3", SynchCompiler = "0.4.3", julia = "1.13")

# Namespace for the emitted package's deterministic UUID, so re-exporting the same app name
# keeps the same package identity, and with it its precompile cache.
const APP_UUID_NAMESPACE = UUID("6f4b3a3e-2f2d-4a1a-9b4c-4d5a5b6c7d8e")

# `@v#.#` (the target Julia's own default environment, where JuliaC is installed) spelled as a
# constant because `#` starts a comment inside a command literal.
const DEFAULT_ENV = "@v#.#"

# ---------------------------------------------------------------------------
## Exporting
# ---------------------------------------------------------------------------
"""
    export_program_juliac(gen, dir; app_name="QubeProgramApp", Tf, arm_deg=0.0,
                          card_options=nothing, gains=(;), julia_channel="1.13",
                          trim="safe", build=true) -> (; app_dir, files, exe, buildlog, ...)

Export a source-generated program (from [`compile_program_source`](@ref)) as a standalone
Julia application under `dir` and, unless `build = false`, compile it into a trimmed binary.

| file | contents |
|:--|:--|
| `<app>/Project.toml`         | the app package, including SynchJulia's `dynamic_execution = false` |
| `<app>/src/controller.jl`    | the code-generated node and its parameter structs |
| `<app>/src/hardware_ffi.jl`  | the operator wrappers the node calls, over the copied `csrc/` |
| `<app>/src/<app>.jl`         | the application: baked-in parameters, the timing loop, `@main` |
| `<app>/csrc/`, `<app>/deps/` | the C the node calls into, and the libraries built from it |
| `<app>/README.md`            | how to build and run it |

`gains` overrides the runtime-settable parameters field by field, exactly as when running
in-process; the values are written into the app as literals. `Tf`, `arm_deg`, `card_options`
and the log's identity are baked in the same way the C harness bakes them.

The built binary is tied to this directory: the operator wrappers `ccall` the libraries in
`<app>/deps` by absolute path, as this package does for its own. Deploying to another host
therefore means exporting and building there, which is what the C target's `make` amounts to
as well.
"""
function export_program_juliac(gen, dir; app_name::AbstractString = "QubeProgramApp",
                               Tf, arm_deg = 0.0, card_options = nothing, gains = (;),
                               julia_channel::AbstractString = APP_COMPAT.julia,
                               trim::AbstractString = "safe", build::Bool = true)
    Base.isidentifier(app_name) ||
        throw(ArgumentError("app_name `$app_name` is not a valid Julia identifier"))
    dir = abspath(dir)
    app_dir = joinpath(dir, app_name)
    for sub in ("src", "csrc", "deps")
        mkpath(joinpath(app_dir, sub))
    end

    # The C the node calls into travels with the app and is built next to it, so the binary
    # does not reach back into this package's `deps`.
    csrc = String[]
    for f in ("qube_hw.c", "qube_hw.h", "qube_log.c", "qube_log.h")
        cp(joinpath(dirname(QUBE_HW_SRC), f), joinpath(app_dir, "csrc", f); force = true)
        push!(csrc, f)
    end
    traj = gen.traj
    if traj !== nothing
        for f in ("qube_traj.c", "qube_traj.h")
            cp(joinpath(dirname(QUBE_HW_SRC), f), joinpath(app_dir, "csrc", f); force = true)
            push!(csrc, f)
        end
        isfile(traj.file) ||
            error("export_program_juliac: the trajectory $(traj.file) does not exist")
        cp(traj.file, joinpath(app_dir, basename(traj.file)); force = true)
        traj = ProgramTrajectory(basename(traj.file), traj.column)
    end
    libs = build_app_libs(app_dir; traj = traj !== nothing)

    write(joinpath(app_dir, "Project.toml"), app_project(app_name))
    write(joinpath(app_dir, "src", "controller.jl"), controller_source(gen.decls))
    write(joinpath(app_dir, "src", "hardware_ffi.jl"),
          emit_program_ffi(gen.operators, libs; traj = traj !== nothing))
    write(joinpath(app_dir, "src", "$app_name.jl"),
          app_source(app_name; gen.Ts, Tf, arm_deg, card_options, gen.log, traj,
                     tuning = tuning_values(gen; gains)))
    write(joinpath(app_dir, "README.md"), app_readme(app_name, gen.log))

    exe = nothing
    buildlog = nothing
    if build
        buildlog = joinpath(dir, "build.log")
        exe = build_program_juliac(app_dir; app_name, julia_channel, trim,
                                   bundle_dir = joinpath(dir, "bundle"), logpath = buildlog)
    end
    files = sort!([relpath(p, app_dir)
                   for p in _walk_files(app_dir) if !startswith(relpath(p, app_dir), "deps")])
    return (; dir, app_dir, files, libs, exe, buildlog, log = gen.log)
end

_walk_files(root) = [joinpath(r, f) for (r, _, fs) in walkdir(root) for f in fs]

"""
    build_app_libs(app_dir; traj=false) -> NamedTuple

Compile the app's own copies of `csrc/qube_hw.c`, `csrc/qube_log.c` (and `qube_traj.c`) into
`<app_dir>/deps`, and return the resulting library paths. Same flags as this package's own
build, so `hil` follows whether the Quanser SDK is installed: without it the binary still
builds and runs against a simulator through the callback backend, but cannot open the board.
"""
function build_app_libs(app_dir; traj::Bool = false)
    cc = _c_compiler()
    cc === nothing && error("""
        No C compiler found (tried cc, gcc, clang and SynchCompiler's Clang_unified_jll).
        One is needed to build the app's copy of the hardware I/O it calls into.""")
    src(f) = joinpath(app_dir, "csrc", f)
    out(name) = joinpath(app_dir, "deps", name * "." * (Sys.iswindows() ? "dll" :
                                                        Sys.isapple() ? "dylib" : "so"))
    base = ["-O2", "-Wall", "-fPIC", "-shared"]
    sdk = quanser_sdk_flags()
    hw = out("libqube_hw")
    hwflags = sdk.found ? vcat(base, ["-DQUBE_HW_HAVE_HIL"], sdk.cflags) : base
    hwlibs = sdk.found ? vcat(sdk.ldflags, QUANSER_LIBS) : String[]
    run(`$cc $hwflags -o $hw $(src("qube_hw.c")) $hwlibs`)
    log = out("libqube_log")
    run(`$cc $base -o $log $(src("qube_log.c"))`)
    tr = nothing
    if traj
        tr = out("libqube_traj")
        run(`$cc $base -o $tr $(src("qube_traj.c"))`)
    end
    return (; hw, log, traj = tr, hil = sdk.found)
end

# ---------------------------------------------------------------------------
## The emitted sources
# ---------------------------------------------------------------------------
function app_project(app_name)
    uuid = uuid5(APP_UUID_NAMESPACE, app_name)
    return """
    # Auto-generated by QuanserComponents.export_program_juliac — do not edit by hand.
    name = "$app_name"
    uuid = "$uuid"
    version = "0.1.0"

    [deps]
    FunctionWrappers = "069b7b12-0de2-55c6-9aab-29f3d0a68a2e"
    StaticArrays = "90137ffa-7385-5640-81b9-e52037218182"
    SynchCompiler = "5d9dccf6-a926-4748-b7e2-6521ccc431d1"
    SynchJulia = "a1b2c3d4-5e6f-7a8b-9c0d-e1f2a3b4c5d6"

    [compat]
    SynchCompiler = "$(APP_COMPAT.SynchCompiler)"
    SynchJulia = "$(APP_COMPAT.SynchJulia)"
    julia = "$(APP_COMPAT.julia)"

    # The executable is built during precompilation and no node definition can change
    # afterwards, so `step!`/`reset!` need no world-age handling. Switching it off is what
    # makes them statically resolvable: the pinned-world fallback, whose
    # `Base.invoke_in_world` the trim verifier rejects, folds away. SynchJulia reads this at
    # its own precompilation, hence a preference rather than a runtime switch.
    [preferences.SynchJulia]
    dynamic_execution = false
    """
end

# Print the code-generated declarations as Julia source. They are emitted unexpanded and
# de-qualified by `compile_program_source` precisely so that they round-trip through `string`;
# the tests parse the result back to check that.
function controller_source(decls)
    io = IOBuffer()
    print(io, """
    # Auto-generated by QuanserComponents.export_program_juliac — do not edit by hand.
    #
    # The program as a SynchJulia synchronous node, code-generated from its Dyad model by
    # SynchToolkit:
    #
    #     (outputs...) = top(tick, gains::TuningGains, auto::AutoPars)
    #
    # The node does its own hardware I/O and its own logging, through the operators in
    # hardware_ffi.jl; the outputs are for inspection. `TuningGains` holds the parameters that
    # stay settable at runtime, `AutoPars` every other model parameter.

    """)
    for ex in decls
        println(io, string(_strip_linenums(ex)))
        println(io)
    end
    return String(take!(io))
end

# Drop source positions so the emitted file neither carries SynchToolkit's own file paths nor
# changes when they move. `Base.remove_linenums!` leaves a macro call's mandatory position
# argument alone (it would print as a `#= … =#` comment), so nil those out too.
function _strip_linenums(ex)
    ex isa Expr || return ex
    ex = Base.remove_linenums!(copy(ex))
    args = Any[_strip_linenums(a) for a in ex.args]
    ex.head === :macrocall && length(args) >= 2 && (args[2] = nothing)
    return Expr(ex.head, args...)
end

# The `csrc` entry points the node calls, by operator name: the Julia name the generated code
# uses, the C symbol behind it, which library it lives in, and its argument count. Mirrors
# src/hardware_io.jl, src/data_log.jl and src/traj_source.jl, which declare the same
# `ccall`s for the in-process backend.
const OPERATOR_FFI = Dict{Symbol, Tuple{Symbol, Symbol, Int}}(
    :hw_measure        => (:qube_hw_measure, :hw, 1),
    :hw_shoulder       => (:qube_hw_shoulder, :hw, 1),
    :hw_elbow          => (:qube_hw_elbow, :hw, 1),
    :hw_write          => (:qube_hw_write, :hw, 2),
    :hw_time           => (:qube_hw_time, :hw, 1),
    :hw_dt             => (:qube_hw_dt, :hw, 1),
    :hw_exec           => (:qube_hw_exec, :hw, 1),
    :hw_count_shoulder => (:qube_hw_count_shoulder, :hw, 1),
    :hw_count_elbow    => (:qube_hw_count_elbow, :hw, 1),
    :log_row           => (:qube_log_row, :log, 8),
    :traj_value        => (:qube_traj_value, :traj, 1),
)

"""
    emit_program_ffi(operators, libs; traj=false) -> String

The app's `src/hardware_ffi.jl`: one `ccall` wrapper per operator the node calls, plus the
driver-side calls its `@main` needs to open and close the device and the log.

The generated node refers to these by bare name (see [`compile_program_source`](@ref)), so
defining them here is what keeps the emitted package independent of QuanserComponents while
still calling the very same C implementations. Library paths are absolute, as they are in
this package: a `ccall` library has to be a compile-time constant.
"""
function emit_program_ffi(operators, libs; traj::Bool = false)
    io = IOBuffer()
    print(io, """
    # Auto-generated by QuanserComponents.export_program_juliac — do not edit by hand.
    #
    # The hardware I/O and logging the generated node calls into: the same csrc/ code this
    # rig's other two targets use, reached by `ccall` exactly as QuanserComponents does it.
    # Paths are absolute because a `ccall` library must be a compile-time constant, which is
    # also why the built binary belongs to the directory it was exported into.

    const QUBE_HW_LIB = $(repr(libs.hw))
    const QUBE_LOG_LIB = $(repr(libs.log))
    """)
    traj && println(io, "const QUBE_TRAJ_LIB = ", repr(libs.traj))
    println(io)
    println(io, "# ---- called by the node, once per tick ----")
    for op in operators
        haskey(OPERATOR_FFI, op) ||
            error("export_program_juliac: the generated node calls `$op`, which has no \
                   known C entry point. Add it to `QuanserComponents.OPERATOR_FFI`.")
        sym, lib, nargs = OPERATOR_FFI[op]
        args = join(("a$i" for i in 1:nargs), ", ")
        types = "(" * repeat("Cdouble, ", nargs) * ")"
        libname = lib === :hw ? "QUBE_HW_LIB" : lib === :log ? "QUBE_LOG_LIB" : "QUBE_TRAJ_LIB"
        println(io, "$op($args) = ccall((:$sym, $libname), Cdouble, $types, $args)")
    end
    print(io, """

    # ---- called by the driver, around the loop ----
    qube_hw_open(mode::Cint, arm_home_rad::Float64) =
        ccall((:qube_hw_open, QUBE_HW_LIB), Cint, (Cint, Cdouble), mode, arm_home_rad)
    qube_hw_close() = ccall((:qube_hw_close, QUBE_HW_LIB), Cvoid, ())
    qube_hw_set_card_options(opts::String) =
        ccall((:qube_hw_set_card_options, QUBE_HW_LIB), Cvoid, (Cstring,), opts)
    qube_log_open(file::String, header::String, ncols::Cint) =
        ccall((:qube_log_open, QUBE_LOG_LIB), Cint, (Cstring, Cstring, Cint),
              file, header, ncols)
    qube_log_close() = ccall((:qube_log_close, QUBE_LOG_LIB), Cvoid, ())
    qube_log_rows() = ccall((:qube_log_rows, QUBE_LOG_LIB), Clong, ())
    qube_log_error() = ccall((:qube_log_error, QUBE_LOG_LIB), Cint, ())
    """)
    if traj
        print(io, """
        qube_traj_open(file::String, column::Cint) =
            ccall((:qube_traj_open, QUBE_TRAJ_LIB), Cint, (Cstring, Cint), file, column)
        qube_traj_close() = ccall((:qube_traj_close, QUBE_TRAJ_LIB), Cvoid, ())
        qube_traj_length() = ccall((:qube_traj_length, QUBE_TRAJ_LIB), Clong, ())
        qube_traj_error() = ccall((:qube_traj_error, QUBE_TRAJ_LIB), Cint, ())
        """)
    end
    return String(take!(io))
end

# The application module: a Julia transcription of csrc/run_hardware.c, and for the same
# reason as that file it reads none of the node's outputs -- the program logs what it wants
# logged. Everything here must stay statically resolvable from `@main`: no logging macros, no
# `Base.stdout` (a non-constant global), and the C reached through the wrappers above.
function app_source(app_name; Ts, Tf, arm_deg, card_options, log::ProgramLog, traj, tuning)
    tunelines = join(("    $(_kwname(k)) = $(_literal(v))," for (k, v) in tuning), "\n")
    # Empty means "do not call `qube_hw_set_card_options`", which leaves qube_hw.c on its own
    # default — the same thing the C harness does when nothing is passed.
    opts = card_options === nothing ? "" : String(card_options)
    trajblock = traj === nothing ? "" : """

        if qube_traj_open($(repr(traj.file)), Cint($(traj.column))) != 0
            print(Core.stderr, "could not read the trajectory\\n")
            qube_log_close(); qube_hw_close()
            return 1
        end
    """
    trajclose = traj === nothing ? "" : """
        qube_traj_error() != 0 &&
            print(Core.stderr, "the run outlived the trajectory (0 V was commanded)\\n")
        qube_traj_close()
    """
    return """
    # Auto-generated by QuanserComponents.export_program_juliac — do not edit by hand.
    module $app_name

    using SynchJulia
    using SynchCompiler   # build-time only: compiles the node during precompilation

    include("hardware_ffi.jl")
    include("controller.jl")

    "Sample time [s]."
    const TS = $(repr(float(Ts)))
    "Run duration [s]."
    const TF = $(repr(float(Tf)))
    "Where the arm physically is at start-up [rad], added to every shoulder reading."
    const ARM0 = $(repr(deg2rad(float(arm_deg))))
    "Card options pinning the command-to-torque path; empty leaves qube_hw.c's own default."
    const CARD_OPTIONS = $(repr(opts))
    "The log the program writes its rows into, and what its columns are called."
    const LOG_FILE = $(repr(log.file))
    const LOG_HEADER = $(repr(join(log.columns, "\\t")))
    const LOG_NCOLS = Cint($(length(log.columns)))
    const HW_MODE_HIL = Cint(1)

    # The program's parameters, resolved at export time.
    const GAINS = TuningGains(;
$tunelines
    )
    # Every other model parameter: the generated constructor computes the ones whose defaults
    # are relations of the tunables.
    const AUTO = AutoPars(GAINS)

    # Constructed at precompilation time and serialized into the package image. This is the
    # whole point of the build-time/run-time split, and it needs Julia >= 1.13.
    const EXE = SynchExecutable(top, (Bool, TuningGains, AutoPars))

    \"\"\"
        run() -> Int

    Open the device and the log, then tick the program every `TS` seconds for `TF` seconds.

    Opening in HIL mode enables the amplifier, zeroes the motor and records the current
    encoder counts as homing offsets. Let the pendulum hang straight down (0 = down, pi = up);
    the arm may be anywhere, as long as `ARM0` says where. A swing-up program's `GoHome` then
    drives the arm to centre.

    One `step!` per tick reads both encoders, computes, writes the motor voltage and appends a
    row to the log — all inside the node. Nothing here reads its outputs.
    \"\"\"
    function run()
        isempty(CARD_OPTIONS) || qube_hw_set_card_options(CARD_OPTIONS)
        if qube_hw_open(HW_MODE_HIL, ARM0) != 0
            print(Core.stderr, "could not open the device\\n")
            return 1
        end
        if qube_log_open(LOG_FILE, LOG_HEADER, LOG_NCOLS) != 0
            print(Core.stderr, "could not open the log for writing\\n")
            qube_hw_close()
            return 1
        end$trajblock
        reset!(EXE)

        # Timing mirrors the C loop and the in-process one: run the body, then sleep the
        # remainder of TS (relative sleep, no absolute-schedule catch-up, so one slow step
        # stretches that period instead of compressing the next ones). The dt/exec measured
        # here are for the summary line only; the program logs its own, from inside the tick.
        n = round(Int, TF / TS)
        t0 = time()
        prev = t0
        sum_dt = 0.0
        max_dt = 0.0
        max_exec = 0.0
        periods = 0
        try
            for i in 1:n
                start = time()
                dt = start - prev
                prev = start
                step!(EXE, true, GAINS, AUTO)
                finish = time()
                exec = finish - start
                if i > 1
                    sum_dt += dt
                    dt > max_dt && (max_dt = dt)
                    periods += 1
                end
                exec > max_exec && (max_exec = exec)
                remain = TS - exec
                remain > 0.0 && Libc.systemsleep(remain)
            end
        finally
            # Zeroes the motor and releases the board, whatever happened.
            qube_hw_close()
            rows = qube_log_rows()
            qube_log_close()$trajclose
            print(Core.stderr, "$app_name: Ts=")
            print(Core.stderr, TS)
            print(Core.stderr, " s | mean dt=")
            print(Core.stderr, periods > 0 ? sum_dt / periods : 0.0)
            print(Core.stderr, " s, max dt=")
            print(Core.stderr, max_dt)
            print(Core.stderr, " s, max exec=")
            print(Core.stderr, max_exec)
            print(Core.stderr, " s over ")
            print(Core.stderr, periods)
            print(Core.stderr, " periods | ")
            print(Core.stderr, rows)
            print(Core.stderr, " rows in ")
            print(Core.stderr, LOG_FILE)
            print(Core.stderr, '\\n')
            qube_log_error() != 0 &&
                print(Core.stderr, "the log was closed by a write error\\n")
        end
        return 0
    end

    function (@main)(ARGS::Vector{String})
        # The node was compiled at build time; never silently fall back to the in-session
        # compiler, which a trimmed binary does not carry.
        SynchJulia.compilation_enabled!(false)
        return run()
    end

    end # module $app_name
    """
end

function app_readme(app_name, log::ProgramLog)
    return """
    # $app_name

    Auto-generated by `QuanserComponents.export_program_juliac` — a QUBE-Servo program
    (code-generated from its Dyad model by SynchToolkit) packaged as a standalone Julia
    application.

    The node is compiled and its `SynchExecutable` constructed during *precompilation*, and so
    serialized into the package image; `@main` only ticks it. That is what makes the runtime
    path compatible with JuliaC's `--trim=safe`.

    ## Build

    Julia $(APP_COMPAT.julia) or newer and JuliaC 0.3.8 or newer are required, the latter
    installed into that Julia's default environment:

    ```sh
    julia +$(APP_COMPAT.julia) -m JuliaC \\
      --output-exe $app_name \\
      --bundle ../bundle \\
      --trim=safe \\
      .
    ```

    ## Run

    ```sh
    ../bundle/bin/$app_name
    ```

    It drives the pendulum for the duration baked in at export time and writes
    `$(basename(log.file))` (columns: $(join(log.columns, ", "))) into its working directory,
    the same log the C target produces.

    Note that `src/hardware_ffi.jl` names the libraries in `deps/` by absolute path, so this
    package and its binary belong to this directory. To deploy elsewhere, export and build
    there.
    """
end

# Emit a value as a Julia literal. `repr` round-trips Float64 exactly, so the deployed
# controller is bit-identical to the in-process one.
_literal(v::AbstractArray) = "[" * join((repr(x) for x in v), ", ") * "]"
_literal(v) = repr(v)

# Parameter names are namespaced with `₊`, a valid identifier character; quote anything else
# so it stays a legal keyword argument.
_kwname(name::Symbol) = Base.isidentifier(name) ? string(name) : "var\"$name\""

# ---------------------------------------------------------------------------
## Building and running
# ---------------------------------------------------------------------------
"""
    build_program_juliac(app_dir; app_name=basename(app_dir), julia_channel="1.13",
                         trim="safe", bundle_dir=..., logpath=nothing) -> exe_path

Compile an emitted application package into a standalone binary: resolve its environment,
then run

    julia +<julia_channel> -m JuliaC --output-exe <app_name> --bundle <bundle_dir> --trim=<trim> <app_dir>

Returns the bundled executable's path. The combined output goes to `logpath` (default
`<dirname(app_dir)>/build.log`) and to stderr; a failing build throws.
"""
function build_program_juliac(app_dir; app_name::AbstractString = basename(abspath(app_dir)),
                              julia_channel::AbstractString = APP_COMPAT.julia,
                              trim::AbstractString = "safe",
                              bundle_dir = joinpath(dirname(abspath(app_dir)), "bundle"),
                              logpath = nothing)
    app_dir = abspath(app_dir)
    bundle_dir = abspath(bundle_dir)
    logpath = something(logpath, joinpath(dirname(app_dir), "build.log"))
    jl = _juliaup_julia(julia_channel)
    open(logpath, "w") do log
        _run_logged(log, _clean_env(`$jl --startup-file=no --project=$app_dir
                                     -e "using Pkg; Pkg.instantiate()"`))
        # JuliaC resolves `--output-exe` against the working directory, so run it in a scratch
        # one: linking fails outright if the caller's cwd holds a directory of that name (the
        # app package itself, say), and nothing is left behind either way.
        _run_logged(log, _clean_env(`$jl --startup-file=no --project=$DEFAULT_ENV -m JuliaC
                                     --output-exe $app_name --bundle $bundle_dir
                                     --trim=$trim $app_dir`); dir = mktempdir())
    end
    exe = joinpath(bundle_dir, "bin", Sys.iswindows() ? "$app_name.exe" : app_name)
    isfile(exe) || error("build_program_juliac: JuliaC produced no executable at $exe \
                          (build log: $logpath)")
    return exe
end

"""
    run_program_juliac(exe) -> log_path

Run a built binary, which drives the pendulum for the duration baked into it and writes its
log into the executable's directory. Returns that log's path.
"""
function run_program_juliac(exe; log_name = SWINGUP_LOG_FILE)
    exe = abspath(exe)
    dir = dirname(exe)
    # The child's working directory is where the program's log lands, so put it next to the
    # binary rather than wherever this process happens to be — as `run_hardware_harness` does.
    run(Cmd(`$exe`; dir))
    return joinpath(dir, basename(log_name))
end

"""
    juliac_available(; julia_channel="1.13") -> Bool

Whether [`build_program_juliac`](@ref) can run here: a `julia +<julia_channel>` of at least
1.13 exists and JuliaC is loadable from that Julia's default environment. For gating tests and
scripts on the toolchain being installed.
"""
function juliac_available(; julia_channel::AbstractString = APP_COMPAT.julia)
    julia = Sys.which("julia")
    julia === nothing && return false
    cmd = _clean_env(`$julia +$julia_channel --startup-file=no --project=$DEFAULT_ENV
                      -e $JULIAC_PROBE`)
    return success(pipeline(cmd; stdout = devnull, stderr = devnull))
end

const JULIAC_PROBE = "VERSION >= v\"1.13-\" || exit(1); using JuliaC"

# `julia +<channel>` is a juliaup shim and `Cmd` does not go through a shell, so resolve the
# channel up front and fail with an actionable message if it is unusable.
function _juliaup_julia(channel)
    juliac_available(; julia_channel = channel) ||
        error("build_program_juliac: no usable `julia +$channel` with JuliaC. Install the \
               channel with `juliaup add $channel` (JuliaC's --trim needs Julia >= 1.13) and \
               JuliaC into its default environment with \
               `julia +$channel -e 'using Pkg; Pkg.add(\"JuliaC\")'`.")
    return `$(Sys.which("julia")) +$channel`
end

# `-m JuliaC` resolves JuliaC from the *active* project, and both `--project=@v#.#` and this
# environment scrub are needed: a caller inside another project — a Dyad analysis run from the
# Builder, or any session started with `--project` — exports `JULIA_PROJECT`, which the child
# would otherwise inherit and where JuliaC is not a dependency.
_clean_env(cmd::Cmd) = addenv(cmd, "JULIA_PROJECT" => nothing, "JULIA_LOAD_PATH" => nothing)

# Run `cmd`, teeing its output into `log` (kept for the analysis' build-log artifact) and onto
# stderr so a long JuliaC build is not silent. The output is written in the `finally` block, so
# a failing build still leaves its diagnostics in the log.
function _run_logged(log, cmd; dir = nothing)
    println(log, "\$ ", cmd)
    flush(log)
    dir === nothing || (cmd = Cmd(cmd; dir))
    out = IOBuffer()
    try
        run(pipeline(cmd; stdout = out, stderr = out))
    finally
        s = String(take!(out))
        write(log, s)
        write(stderr, s)
        flush(log)
    end
    return nothing
end
