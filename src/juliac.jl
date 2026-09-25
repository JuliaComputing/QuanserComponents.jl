# Deploying a program as a standalone binary compiled by JuliaC.
#
# The JuliaC counterpart of the C export in harness.jl. The deployed program is the same -- the
# node does its own hardware I/O and its own logging, so what surrounds it is only a timing
# loop -- and so is the arrangement:
#
#   export_program_c       top.c + csrc + run_hardware.c        -> `make`  -> ./run_hardware
#   export_program_juliac  an application package + csrc        -> JuliaC  -> bundle/bin/<App>
#
# What makes a trimmed build possible is SynchJulia's split between construction and
# execution (docs/src/man/deployment.md in SynchJulia): the emitted package defines the node and
# constructs its `SynchExecutable` at top level, so both happen during precompilation and are
# serialized into the package image, and `@main` only calls `step!` and `reset!`. The
# `dynamic_execution = false` preference removes the world-age check from those two, which
# leaves the trim verifier with direct calls.
#
# JuliaC cannot cross-compile: the package image is produced by the Julia that runs the build.
# A binary for another architecture is therefore built by that architecture's Julia, which on
# this machine means aarch64 Julia under qemu-user, inside the rootless root file system that
# deploy/arm64/setup.sh prepares. Everything the build needs is inside the application
# directory (the C sources and a copy of SynchJulia), so the same commands run in either place.

export export_program_juliac, build_program_juliac, deploy_juliac_bundle, run_juliac!,
       juliac_available

# JuliaC's `--trim` needs Julia 1.13, and so does a node cached in a package image.
const JULIAC_MIN_JULIA = v"1.13"

# The environment on this machine that JuliaC is installed into (`julia --project=@juliac`).
const JULIAC_ENV = "@juliac"

# Where deploy/arm64/setup.sh installs JuliaC inside the arm64 root file system.
const JULIAC_ENV_ARM64 = "/opt/juliac"

const DEPLOY_DIR = normpath(joinpath(@__DIR__, "..", "deploy"))

# The platforms a binary can be built for: this machine, or a 64-bit Raspberry Pi. The CPU
# target is fixed for the latter: under qemu, the native CPU is qemu's own model, whose
# features (SVE among them) the Pi 4's Cortex-A72 does not have.
const JULIAC_PLATFORMS = Dict("" => (; cpu_target = nothing),
                              "linux/arm64" => (; cpu_target = "cortex-a72"))

# ---------------------------------------------------------------------------
## Exporting
# ---------------------------------------------------------------------------
"""
    export_program_juliac(src::ProgramSource, dir; Tf, arm_deg=0.0, card_options=nothing,
                          gains=(;), app_name=juliac_app_name(src)) -> (; dir, app_dir, app_name, files)

Write a source-compiled program (from [`compile_program_source`](@ref)) as a standalone Julia
application into `dir/<app_name>`, ready for [`build_program_juliac`](@ref):

| file | contents |
|:--|:--|
| `Project.toml`         | the package, depending on SynchJulia only (and what the generated code imports) |
| `LocalPreferences.toml`| SynchJulia's `dynamic_execution = false` |
| `src/controller.jl`    | the code-generated node and its parameter structs |
| `src/hardware_ffi.jl`  | the operators the node calls, as `ccall`s into the libraries built from `csrc/` |
| `src/<app_name>.jl`    | the baked-in parameters, the executable, the timing loop and `@main` |
| `csrc/`                | `qube_hw`, `qube_log`, `qube_traj` and `libs.mk`, which builds them |
| `vendor/SynchJulia`    | the SynchJulia the node was generated with |

SynchJulia is copied into the package because it is registered only in the private
DyadRegistry: with the copy, the build needs nothing but the General registry, which is what a
fresh root file system has. It also guarantees that the node is compiled by the SynchJulia
that generated it.

`gains` overrides the runtime-settable parameters field by field, as for every other target,
and the values are written into the application as literals. `Tf`, `arm_deg`, `card_options`
and the log's identity are baked in the same way the C harness bakes them. A replayed
trajectory is copied into `dir`, the directory the program runs in.
"""
function export_program_juliac(src::ProgramSource, dir; Tf, arm_deg = 0.0,
                               card_options = nothing, gains = (;),
                               app_name::AbstractString = juliac_app_name(src))
    Base.isidentifier(app_name) ||
        throw(ArgumentError("app_name `$app_name` is not a valid Julia identifier"))
    dir = abspath(dir)
    app_dir = joinpath(dir, app_name)
    # Start from nothing, so no file of an earlier export (a trajectory library, say) survives.
    rm(app_dir; recursive = true, force = true)
    for sub in ("src", "csrc", "vendor")
        mkpath(joinpath(app_dir, sub))
    end
    csrc = dirname(QUBE_HW_SRC)
    for f in ("qube_hw.c", "qube_hw.h", "qube_log.c", "qube_log.h", "qube_traj.c",
              "qube_traj.h", "libs.mk")
        cp(joinpath(csrc, f), joinpath(app_dir, "csrc", f); force = true)
    end
    cp(pkgdir(SynchJulia), joinpath(app_dir, "vendor", "SynchJulia"))
    traj = src.traj
    if traj !== nothing
        isfile(traj.file) ||
            error("export_program_juliac: the trajectory $(traj.file) does not exist")
        cp(traj.file, joinpath(dir, basename(traj.file)); force = true)
        traj = ProgramTrajectory(basename(traj.file), traj.column)
    end
    write(joinpath(app_dir, "Project.toml"), app_project(app_name))
    write(joinpath(app_dir, "LocalPreferences.toml"), APP_PREFERENCES)
    write(joinpath(app_dir, "src", "controller.jl"), controller_source(src.decls))
    write(joinpath(app_dir, "src", "hardware_ffi.jl"), emit_program_ffi(src.operators))
    write(joinpath(app_dir, "src", "$app_name.jl"),
          app_source(app_name; src.Ts, Tf, arm_deg, card_options, src.log, traj,
                     tuning = tuning_values(src; gains)))
    # Files copied out of the read-only package depot keep their mode; everything here is a
    # build output, so leave nothing unwritable for the next export's `rm`.
    for (root, _, fs) in walkdir(app_dir), f in fs
        p = joinpath(root, f)
        m = filemode(p)
        (m & 0o200) == 0 && chmod(p, m | 0o200)
    end
    files = sort!([relpath(joinpath(root, f), dir)
                   for (root, _, fs) in walkdir(app_dir) for f in fs
                   if !startswith(relpath(root, app_dir), "vendor")])
    traj === nothing || push!(files, traj.file)
    return (; dir, app_dir, app_name, files)
end

"""
    juliac_app_name(src::ProgramSource) -> String

The application name a program is exported under by default: `Qube` followed by the
program's system name, e.g. `QubeController` for the swing-up controller.
"""
juliac_app_name(src::ProgramSource) =
    "Qube" * join(uppercasefirst.(split(string(src.spec.name), '_')))

# ---------------------------------------------------------------------------
## The emitted sources
# ---------------------------------------------------------------------------
const GENERATED_HEADER = "# Generated by QuanserComponents.export_program_juliac. Do not edit."

# UUIDs of what the generated code imports (SynchToolkit's `using StaticArrays,
# FunctionWrappers, SynchJulia`), which the package therefore has to depend on.
const APP_DEPS = ("FunctionWrappers" => "069b7b12-0de2-55c6-9aab-29f3d0a68a2e",
                  "StaticArrays" => "90137ffa-7385-5640-81b9-e52037218182",
                  "SynchJulia" => "a1b2c3d4-5e6f-7a8b-9c0d-e1f2a3b4c5d6")

function app_project(app_name)
    # Deterministic, so that re-exporting the same application keeps its precompile cache.
    uuid = Base.UUID(UInt128(hash(app_name, 0x51d1f0ab5eed0001)) << 64 |
                     hash(app_name, 0x51d1f0ab5eed0002))
    deps = join(("$k = \"$v\"" for (k, v) in APP_DEPS), "\n")
    return """
    $GENERATED_HEADER
    name = "$app_name"
    uuid = "$uuid"
    version = "0.1.0"

    [deps]
    $deps

    [sources]
    SynchJulia = {path = "vendor/SynchJulia"}

    [compat]
    julia = "$(JULIAC_MIN_JULIA.major).$(JULIAC_MIN_JULIA.minor)"
    """
end

# The executable is built during precompilation, and no definition can change afterwards.
const APP_PREFERENCES = """
$GENERATED_HEADER
# The executable is constructed during precompilation and no definition can change afterwards,
# so `step!` and `reset!` need no world-age check. SynchJulia reads this when it is itself
# precompiled.
[SynchJulia]
dynamic_execution = false
"""

# Print the generated declarations as Julia source. `compile_program_source` leaves the node
# unexpanded and the operators unqualified precisely so that they round-trip through `string`;
# the tests parse the result back.
function controller_source(decls)
    io = IOBuffer()
    print(io, """
    $GENERATED_HEADER
    #
    # The program as a SynchJulia node, code-generated from its Dyad model by SynchToolkit:
    #
    #     (outputs...) = top(tick, gains::TuningGains, auto::AutoPars)
    #
    # The node does its own hardware I/O and logging through the operators in hardware_ffi.jl;
    # its outputs are not read. `TuningGains` holds the parameters that stay settable at
    # runtime, `AutoPars` every other model parameter.

    """)
    for ex in decls
        println(io, string(_strip_linenums(ex)))
        println(io)
    end
    return String(take!(io))
end

# Drop source positions, so the emitted file neither carries SynchToolkit's file paths nor
# changes when they move. `Base.remove_linenums!` keeps a macro call's position argument,
# which would print as a `#= ... =#` comment, so those are removed as well.
function _strip_linenums(ex)
    ex isa Expr || return ex
    ex = Base.remove_linenums!(copy(ex))
    args = Any[_strip_linenums(a) for a in ex.args]
    ex.head === :macrocall && length(args) >= 2 && (args[2] = nothing)
    return Expr(ex.head, args...)
end

# The C entry point behind each operator the generated code can call: its symbol, the library
# it is in, and its number of arguments, all of which are `Cdouble`, as is the result. The
# same `ccall`s as in hardware_io.jl, data_log.jl and traj_source.jl, which the tests check
# against this table.
const OPERATOR_FFI = Dict{Symbol, Tuple{Symbol, String, Int}}(
    :hw_measure        => (:qube_hw_measure, "libqube_hw", 1),
    :hw_shoulder       => (:qube_hw_shoulder, "libqube_hw", 1),
    :hw_elbow          => (:qube_hw_elbow, "libqube_hw", 1),
    :hw_write          => (:qube_hw_write, "libqube_hw", 2),
    :hw_time           => (:qube_hw_time, "libqube_hw", 1),
    :hw_dt             => (:qube_hw_dt, "libqube_hw", 1),
    :hw_exec           => (:qube_hw_exec, "libqube_hw", 1),
    :hw_count_shoulder => (:qube_hw_count_shoulder, "libqube_hw", 1),
    :hw_count_elbow    => (:qube_hw_count_elbow, "libqube_hw", 1),
    :hw_realtime_wait  => (:qube_hw_realtime_wait, "libqube_hw", 2),
    :log_row           => (:qube_log_row, "libqube_log", QUBE_LOG_MAX_COLS),
    :traj_value        => (:qube_traj_value, "libqube_traj", 1),
)

"""
    emit_program_ffi(operators) -> String

The application's `src/hardware_ffi.jl`: a `ccall` wrapper for each operator the node calls,
and the calls its `@main` makes to open and close the device, the log and the trajectory.

The libraries are named without a path. `build_program_juliac` copies them into the bundle's
`lib` directory, which the dynamic loader searches for a library the Julia runtime opens (the
run path of `lib/julia/libjulia-internal` includes its parent directory), so the bundle can be
moved to another machine as a whole.
"""
function emit_program_ffi(operators)
    io = IOBuffer()
    print(io, """
    $GENERATED_HEADER
    #
    # The hardware I/O, logging and trajectory replay the generated node calls into: the same
    # csrc/ code the other targets use, built into the libraries in the bundle's lib/.

    const QUBE_HW_LIB = "libqube_hw"
    const QUBE_LOG_LIB = "libqube_log"
    const QUBE_TRAJ_LIB = "libqube_traj"

    # Called by the node, once per tick.
    """)
    for op in operators
        haskey(OPERATOR_FFI, op) ||
            error("export_program_juliac: the generated node calls `$op`, which has no known \
                   C entry point; add it to `QuanserComponents.OPERATOR_FFI`")
        sym, lib, nargs = OPERATOR_FFI[op]
        args = join(("a$i" for i in 1:nargs), ", ")
        types = "(" * repeat("Cdouble, ", nargs) * ")"
        println(io, "$op($args) = ccall((:$sym, $(repr(lib))), Cdouble, $types, $args)")
    end
    print(io, """

    # Called by the application, around the loop.
    qube_hw_open(mode::Cint, arm_home_rad::Float64) =
        ccall((:qube_hw_open, QUBE_HW_LIB), Cint, (Cint, Cdouble), mode, arm_home_rad)
    qube_hw_close() = ccall((:qube_hw_close, QUBE_HW_LIB), Cvoid, ())
    qube_hw_set_card_options(opts::String) =
        ccall((:qube_hw_set_card_options, QUBE_HW_LIB), Cvoid, (Cstring,), opts)
    qube_log_open(file::String, header::String, ncols::Cint) =
        ccall((:qube_log_open, QUBE_LOG_LIB), Cint, (Cstring, Cstring, Cint), file, header, ncols)
    qube_log_close() = ccall((:qube_log_close, QUBE_LOG_LIB), Cvoid, ())
    qube_log_rows() = ccall((:qube_log_rows, QUBE_LOG_LIB), Clong, ())
    qube_log_error() = ccall((:qube_log_error, QUBE_LOG_LIB), Cint, ())
    qube_traj_open(file::String, column::Cint) =
        ccall((:qube_traj_open, QUBE_TRAJ_LIB), Cint, (Cstring, Cint), file, column)
    qube_traj_close() = ccall((:qube_traj_close, QUBE_TRAJ_LIB), Cvoid, ())
    qube_traj_length() = ccall((:qube_traj_length, QUBE_TRAJ_LIB), Clong, ())
    qube_traj_error() = ccall((:qube_traj_error, QUBE_TRAJ_LIB), Cint, ())
    """)
    return String(take!(io))
end

# The application module: a Julia transcription of csrc/run_hardware.c, which for the same
# reason reads none of the node's outputs. Everything reachable from `@main` has to resolve
# statically: no logging macros, no `stdout` (a non-constant global), only `Core.stderr`.
function app_source(app_name; Ts, Tf, arm_deg, card_options, log::ProgramLog, traj, tuning)
    tunelines = join(("    $(_kwname(k)) = $(_literal(v))," for (k, v) in tuning), "\n")
    # Empty means that `qube_hw_set_card_options` is not called, which leaves qube_hw.c on its
    # own default, as the C harness does when no options are given.
    opts = card_options === nothing ? "" : String(card_options)
    trajopen = traj === nothing ? "" : """

            if qube_traj_open($(repr(traj.file)), Cint($(traj.column))) != 0
                print(Core.stderr, "$app_name: could not read the trajectory $(traj.file)\\n")
                qube_log_close()
                qube_hw_close()
                return 1
            end"""
    trajclose = traj === nothing ? "" : """

                qube_traj_error() != 0 &&
                    print(Core.stderr, "$app_name: the run outlived the trajectory (0 V was commanded)\\n")
                qube_traj_close()"""
    return """
    $GENERATED_HEADER
    module $app_name

    using SynchJulia

    include("hardware_ffi.jl")
    include("controller.jl")

    const TS = $(repr(float(Ts)))
    const TF = $(repr(float(Tf)))
    # The arm angle at start-up [rad], added to every shoulder reading.
    const ARM0 = $(repr(deg2rad(float(arm_deg))))
    const CARD_OPTIONS = $(repr(opts))
    const LOG_FILE = $(repr(log.file))
    const LOG_HEADER = $(repr(join(log.columns, "\t")))
    const LOG_NCOLS = Cint($(length(log.columns)))
    const HW_MODE_CALLBACK = Cint(0)
    const HW_MODE_HIL = Cint(1)

    # The runtime-settable parameters, resolved when the application was exported.
    const GAINS = TuningGains(;
    $tunelines
    )
    # Every other model parameter; the generated constructor computes those whose defaults are
    # expressions of the tunable ones.
    const AUTO = AutoPars(GAINS)

    # Constructed during precompilation and serialized into the package image, so that the
    # binary never compiles.
    const EXE = SynchExecutable(top, (Bool, TuningGains, AutoPars))

    \"\"\"
        run(mode) -> Int

    Open the device and the log, then tick the program every `TS` seconds for `TF` seconds.

    Opening in HIL mode enables the amplifier, zeroes the motor and records the current encoder
    counts as the homing offsets, so the pendulum has to hang straight down at start-up; the
    arm may be anywhere, as long as `ARM0` states where. One `step!` reads both encoders,
    computes, writes the motor voltage and appends a row to the log, all inside the node.

    In callback mode no device is opened: with no handlers installed, the encoders read zero
    and motor commands are discarded, so the loop and the log run without hardware.
    \"\"\"
    function run(mode::Cint)
        isempty(CARD_OPTIONS) || qube_hw_set_card_options(CARD_OPTIONS)
        if qube_hw_open(mode, ARM0) != 0
            print(Core.stderr, "$app_name: could not open the device\\n")
            return 1
        end
        if qube_log_open(LOG_FILE, LOG_HEADER, LOG_NCOLS) != 0
            print(Core.stderr, "$app_name: could not open $(log.file) for writing\\n")
            qube_hw_close()
            return 1
        end$trajopen
        reset!(EXE)

        # The timing of csrc/run_hardware.c: run the body, then sleep for the remainder of TS
        # (a relative sleep, so one slow step lengthens that period and does not shorten the
        # following ones). The dt and exec measured here are for the summary line only; the
        # program logs its own from inside the tick.
        n = round(Int, TF / TS)
        prev = time()
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
                exec = time() - start
                if i > 1
                    sum_dt += dt
                    max_dt = max(max_dt, dt)
                    periods += 1
                end
                max_exec = max(max_exec, exec)
                remain = TS - exec
                remain > 0.0 && Libc.systemsleep(remain)
            end
        finally
            # Zeroes the motor and releases the board, whatever happened.
            qube_hw_close()
            rows = qube_log_rows()
            qube_log_close()$trajclose
            # One value per `print`: a long varargs call is not resolved statically.
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
                print(Core.stderr, "$app_name: the log was closed by a write error\\n")
        end
        return 0
    end

    # `$app_name --dry-run` runs without the device (see `run`).
    function (@main)(ARGS::Vector{String})
        # The node was compiled when the package was precompiled; a trimmed binary carries no
        # compiler to fall back on.
        SynchJulia.compilation_enabled!(false)
        dry = any(==("--dry-run"), ARGS)
        dry && print(Core.stderr, "$app_name: dry run, the device is not opened\\n")
        return run(dry ? HW_MODE_CALLBACK : HW_MODE_HIL)
    end

    end # module $app_name
    """
end

# A value as a Julia literal. `repr` round-trips a Float64 exactly, so the deployed program
# computes with the same parameters as the in-process one.
_literal(v::AbstractArray) = "[" * join((repr(x) for x in v), ", ") * "]"
_literal(v) = repr(v)

# Namespaced parameter names contain `₊`, which is an identifier character; anything else is
# quoted so that it remains a valid keyword argument.
_kwname(name::Symbol) = Base.isidentifier(name) ? string(name) : "var\"$name\""

# ---------------------------------------------------------------------------
## Building
# ---------------------------------------------------------------------------
"""
    build_program_juliac(app_dir; platform="", trim="safe", bundle_dir=<dir>/bundle,
                         logpath=<dir>/build.log, cpu_target) -> exe

Compile an exported application (see [`export_program_juliac`](@ref)) into a bundle with
JuliaC, and return the path of the executable, `bundle_dir/bin/<app>`. `<dir>` is the
directory the application was exported into.

The build builds the libraries in `csrc/` with `libs.mk`, instantiates the package, runs

    julia -m JuliaC --output-exe <app> --bundle <bundle_dir> --trim=<trim> <app_dir>

and copies the libraries into `bundle_dir/lib`, where the binary finds them.

`platform` is `""` for this machine or `"linux/arm64"` for a 64-bit Raspberry Pi. The latter
runs the same steps in the arm64 root file system of deploy/arm64/setup.sh, through qemu-user,
which is slower: the swing-up controller takes about six minutes. `cpu_target` defaults to the
platform's (`cortex-a72` for `linux/arm64`, Julia's own default otherwise) and is passed to
JuliaC as `JULIA_CPU_TARGET`.

On this machine the build needs Julia 1.13 or newer as `julia` and JuliaC installed in the
`@juliac` environment; [`juliac_available`](@ref) states whether both are present. Output goes
to `logpath` and to stderr, and a failing build throws.
"""
function build_program_juliac(app_dir; platform::AbstractString = "",
                              trim::AbstractString = "safe",
                              bundle_dir = joinpath(dirname(abspath(app_dir)), "bundle"),
                              logpath = joinpath(dirname(abspath(app_dir)), "build.log"),
                              cpu_target = _juliac_platform(platform).cpu_target)
    app_dir = abspath(app_dir)
    bundle_dir = abspath(bundle_dir)
    app_name = basename(app_dir)
    juliac_available(; platform) ||
        error("build_program_juliac: " * _juliac_missing_message(platform))
    rm(bundle_dir; recursive = true, force = true)
    # JuliaC resolves `--output-exe` against its working directory and fails when that holds a
    # directory of the same name (the application package, say), so it runs in a scratch one.
    # Inside the root file system that has to be a directory shared with the host.
    scratch = mktempdir(dirname(app_dir))
    csrc = joinpath(app_dir, "csrc")
    env = cpu_target === nothing ? () : ("JULIA_CPU_TARGET" => cpu_target,)
    steps = [`make -C $csrc -f libs.mk`,
             `julia --startup-file=no --project=$app_dir -e "using Pkg; Pkg.instantiate()"`,
             Cmd(`julia --startup-file=no --project=$(_juliac_env(platform)) -m JuliaC
                  --output-exe $app_name --bundle $bundle_dir --trim=$trim $app_dir`;
                 dir = scratch)]
    try
        open(logpath, "w") do log
            for step in steps
                _run_logged(log, _in_platform(step, platform, (dirname(app_dir),); env))
            end
        end
    finally
        rm(scratch; recursive = true, force = true)
    end
    for lib in ("libqube_hw.so", "libqube_log.so", "libqube_traj.so")
        cp(joinpath(csrc, lib), joinpath(bundle_dir, "lib", lib); force = true)
    end
    exe = joinpath(bundle_dir, "bin", app_name)
    isfile(exe) ||
        error("build_program_juliac: JuliaC produced no executable at $exe (build log: $logpath)")
    return exe
end

"""
    juliac_available(; platform="") -> Bool

Whether [`build_program_juliac`](@ref) can build for `platform` here. For this machine: `julia`
is at least Julia 1.13 and JuliaC loads in the `@juliac` environment. For `"linux/arm64"`: the
root file system of deploy/arm64/setup.sh exists, with JuliaC installed in it.
"""
function juliac_available(; platform::AbstractString = "")
    _juliac_platform(platform)
    if isempty(platform)
        Sys.which("julia") === nothing && return false
        probe = "VERSION >= v\"$JULIAC_MIN_JULIA\" || exit(1); using JuliaC"
        cmd = _clean_env(`julia --startup-file=no --project=$JULIAC_ENV -e $probe`)
        return success(pipeline(cmd; stdout = devnull, stderr = devnull))
    end
    return isfile(joinpath(arm64_rootfs(), "opt", "juliac", "Manifest.toml")) &&
           isfile(joinpath(arm64_rootfs(), "opt", "julia", "bin", "julia"))
end

"""
    arm64_rootfs() -> String

The arm64 root file system prepared by deploy/arm64/setup.sh: `\$QUBE_ARM64_ROOTFS`, or
`~/.cache/QuanserComponents/arm64-rootfs` when that is not set.
"""
arm64_rootfs() = get(ENV, "QUBE_ARM64_ROOTFS",
                     joinpath(homedir(), ".cache", "QuanserComponents", "arm64-rootfs"))

function _juliac_platform(platform)
    haskey(JULIAC_PLATFORMS, platform) ||
        throw(ArgumentError("platform must be one of \
                             $(join(repr.(sort!(collect(keys(JULIAC_PLATFORMS)))), ", ")), \
                             got $(repr(platform))"))
    return JULIAC_PLATFORMS[platform]
end

_juliac_env(platform) = isempty(platform) ? JULIAC_ENV : JULIAC_ENV_ARM64

_juliac_missing_message(platform) = isempty(platform) ?
    "no Julia $JULIAC_MIN_JULIA or newer with JuliaC is available as `julia`. Install JuliaC \
     with `julia --project=$JULIAC_ENV -e 'using Pkg; Pkg.add(\"JuliaC\")'`." :
    "no arm64 root file system with JuliaC at $(arm64_rootfs()). Prepare it with \
     $(joinpath(DEPLOY_DIR, "arm64", "setup.sh")) (no root privileges needed)."

# `cmd` as run on `platform`: here, or in the arm64 root file system with the directories in
# `binds` shared at the same paths, so that the paths in `cmd` mean the same on both sides.
function _in_platform(cmd::Cmd, platform, binds; env = ())
    isempty(platform) && return addenv(_clean_env(cmd), env...)
    # The variables `run.sh` sets itself (the depot among them) take precedence over these.
    run_sh = joinpath(DEPLOY_DIR, "arm64", "run.sh")
    bindargs = collect(Iterators.flatten(("--bind", b) for b in binds))
    dir = isempty(cmd.dir) ? nothing : cmd.dir
    envargs = ["$k=$v" for (k, v) in env]
    inner = dir === nothing ? `env $envargs $(cmd.exec)` :
            `sh -c 'cd "$1" && shift && exec env "$@"' sh $dir $envargs $(cmd.exec)`
    return addenv(_clean_env(`$run_sh $bindargs $inner`), "QUBE_ARM64_ROOTFS" => arm64_rootfs())
end

# `-m JuliaC` resolves JuliaC from the active project, and a caller inside another project (a
# Dyad analysis run from the Builder, or any session started with `--project`) exports
# `JULIA_PROJECT`, which the child would otherwise inherit.
_clean_env(cmd::Cmd) = addenv(cmd, "JULIA_PROJECT" => nothing, "JULIA_LOAD_PATH" => nothing,
                              "JULIA_DEPOT_PATH" => nothing)

# Run `cmd` with its output going both into `log`, kept as the analyses' build log, and onto
# stderr, so that a long build is not silent.
function _run_logged(log, cmd)
    # The command without its environment, which holds whatever this process was started with.
    println(log, "\$ ", join(cmd.exec, " "))
    flush(log)
    tee = Base.BufferStream()
    task = @async for line in eachline(tee; keep = true)
        write(log, line)
        write(stderr, line)
    end
    try
        run(pipeline(cmd; stdout = tee, stderr = tee))
    finally
        close(tee)
        wait(task)
        flush(log)
    end
    return nothing
end

# ---------------------------------------------------------------------------
## Running
# ---------------------------------------------------------------------------
"""
    run_juliac!(src::ProgramSource; Tf, output_dir="furuta_juliac", platform="", arm_deg=0.0,
                card_options=nothing, deploy_host="", deploy_dir="furuta_juliac",
                live_plot=false, live_plot_cmd="kst2", live_plot_config="kst2config.kst",
                gains=(;), run=true) -> HardwareRun

Export a source-compiled program as a JuliaC application into `output_dir`, build it for
`platform` and, with `run`, run the binary for `Tf` seconds: on this machine, or with a
`deploy_host` on the machine the QUBE is attached to, from which the log is fetched back into
`output_dir`. The JuliaC counterpart of [`run_c!`](@ref), with the same keywords.

A binary for a Raspberry Pi is built here with `platform = "linux/arm64"` (see
[`build_program_juliac`](@ref)), and only the bundle is copied to the host. The host needs
the Quanser SDK installed, as it does for the C target, and no Julia.

The program runs in `output_dir` (`deploy_dir` on the host), which is where it writes its log
and reads a trajectory from.
"""
function run_juliac!(src::ProgramSource; Tf, output_dir = "furuta_juliac",
                     platform::AbstractString = "", arm_deg = 0.0,
                     card_options::Union{Nothing, AbstractString} = nothing,
                     deploy_host::AbstractString = "", deploy_dir = "furuta_juliac",
                     live_plot::Bool = false, live_plot_cmd = "kst2",
                     live_plot_config = "kst2config.kst", gains = (;), run::Bool = true)
    remote = !isempty(deploy_host)
    if remote && isempty(platform) && Sys.ARCH !== :aarch64
        @warn "Deploying a binary built for this machine ($(Sys.ARCH)) to $deploy_host; pass \
               platform = \"linux/arm64\" to build one for a Raspberry Pi"
    end
    res = export_program_juliac(src, output_dir; Tf, arm_deg, card_options, gains)
    exe = build_program_juliac(res.app_dir; platform)
    files = [res.files; relpath(exe, res.dir); "build.log"]
    no_timing = (; median_dt = NaN, max_dt = NaN)
    run || return HardwareRun(false, :none, nothing, 0, 0, no_timing, res.dir, files,
                              res.app_name, nothing)
    log_name = basename(src.log.file)
    expected = round(Int, Tf / src.Ts)
    start_plot() = live_plot ? launch_live_plot(output_dir; cmd = live_plot_cmd,
                                                config = live_plot_config, log = log_name) :
                   nothing
    if remote
        deploy_juliac_bundle(res.dir; host = deploy_host, remote_dir = deploy_dir,
                             traj = src.traj)
        task = @async run_c_harness_remote(deploy_host, deploy_dir; local_dir = output_dir,
                                           log_name, stream_log = live_plot,
                                           exe = "./bundle/bin/$(res.app_name)")
        plotter = start_plot()
        log = fetch(task)
        return HardwareRun(true, :remote_juliac, log, _log_rows(log), expected,
                           log_timing(log), res.dir, files, res.app_name, plotter)
    end
    csv = joinpath(res.dir, log_name)
    rm(csv; force = true)
    task = @async (Base.run(Cmd(`$exe`; dir = res.dir)); csv)
    plotter = start_plot()
    log = fetch(task)
    return HardwareRun(true, :local_juliac, log, _log_rows(log), expected, log_timing(log),
                       res.dir, files, res.app_name, plotter)
end

"""
    deploy_juliac_bundle(dir; host, remote_dir="furuta_juliac", traj=nothing, ssh=`ssh`, scp=`scp`) -> remote_dir

Copy the bundle built in `dir` to `remote_dir` on `host`, replacing an earlier one, together
with the trajectory the program replays, if any. Nothing is built on the host.
"""
function deploy_juliac_bundle(dir; host, remote_dir = "furuta_juliac", traj = nothing,
                              ssh = `ssh`, scp = `scp`)
    bundle = joinpath(dir, "bundle")
    isdir(bundle) || error("deploy_juliac_bundle: no bundle in $dir; build it first")
    @info "Copying the bundle to $host:$remote_dir"
    # Through `tar` rather than `scp -r`, which would copy every symbolic link in `lib/` as a
    # second copy of the library it points to.
    run(pipeline(`tar -C $dir -czf - bundle`,
                 `$ssh $host "mkdir -p $remote_dir && rm -rf $remote_dir/bundle && tar -C $remote_dir -xzf -"`))
    traj === nothing || run(`$scp $(joinpath(dir, basename(traj.file))) $host:$remote_dir/`)
    return remote_dir
end
