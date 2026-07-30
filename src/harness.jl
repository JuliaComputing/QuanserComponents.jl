# Getting a compiled program onto hardware: standalone C export, the timing loop around it,
# deployment to another machine, live plotting, and the choice between the four.
#
# Nothing here knows which program it is carrying. That became possible once the log moved
# inside the program (`DataLogger`): the harness no longer has to name the node's outputs, so
# `run_hardware.c` is a bare timing loop parameterized by nothing but the mangled node
# symbols, the period, the duration and the log's identity — which the swing-up controller and
# the friction experiment supply alike.

using Printf: @printf

export export_program_c, emit_hardware_harness, compile_hardware_harness,
       launch_live_plot, deploy_hardware_harness, run_hardware_harness,
       run_hardware_harness_remote, run_on_target, HardwareRun

# ---------------------------------------------------------------------------
## C export
# ---------------------------------------------------------------------------
# Raw little-endian bytes of a runtime parameter object (`TuningGains`/`AutoPars`), packed
# into 64-bit words. Both are mutable structs whose fields are stored inline, so
# `pointer_from_objref` points at the flat field-data region the exported C reads via its
# baked-in `fieldoffset`s. Packing to `uint64_t` guarantees the 8-byte alignment the C
# pointer casts require (the data region is a whole number of 8-byte fields; pad defensively).
function _param_words(obj)
    n = sizeof(typeof(obj))
    npad = cld(n, 8) * 8
    bytes = zeros(UInt8, npad)
    GC.@preserve obj bytes begin
        unsafe_copyto!(pointer(bytes), Ptr{UInt8}(pointer_from_objref(obj)), n)
    end
    return reinterpret(UInt64, bytes)
end

# Files an export writes that the target needs in order to build and run.
const HARNESS_FILES = ("run_hardware.c", "run_hardware_config.h", "Makefile",
                       "top.c", "top.h", "synchjulia.h", "qube_hw.c", "qube_hw.h",
                       "qube_log.c", "qube_log.h")

"""
    export_program_c(gen, dir; Tf, arm_deg=0.0, card_options=nothing, gains=(;))

Export a compiled program as standalone C into `dir`: the SynchToolkit node (`top.c`,
`top.h`, `top.pc`, `synchjulia.h`), the C the node calls into (`qube_hw.c` for the device,
`qube_log.c` for the log, copied from `csrc/`), and a runnable control loop (`run_hardware.c`,
`Makefile`) — see [`emit_hardware_harness`](@ref).

`gains` overrides the runtime-settable parameters field by field, exactly as when running
in-process; the resulting struct bytes are what gets embedded. `Tf`, `arm_deg` and
`card_options` are baked into the harness.

The exported `top.c` contains `extern double qube_hw_measure(double arg1);` and friends plus
named call sites, so the node performs its own encoder reads, motor writes and log rows, and
the harness is only a timing loop. That is the same code path the Julia backend takes — one
program definition, one implementation of each side effect, three targets.

Returns `(; dir, mangled, files, gains, auto)`, where `mangled` is the base symbol name of
the emitted `<mangled>_step` / `<mangled>_reset` functions.
"""
function export_program_c(gen, dir; Tf, arm_deg = 0.0, card_options = nothing, gains = (;))
    mkpath(dir)
    r = instantiate(gen; gains, export_dir = dir)
    mangled = SynchCompiler.mangle("top", Bool, r.SG, r.AP)
    for f in ("qube_hw.c", "qube_hw.h", "qube_log.c", "qube_log.h")
        cp(joinpath(dirname(QUBE_HW_SRC), f), joinpath(dir, f); force = true)
    end
    emit_hardware_harness(dir; gen.Ts, Tf, arm_deg, card_options, mangled, gen.log,
                          r.gains, r.auto)
    # `export_c` copies `synchjulia.h` out of the package depot, which is read-only, and
    # preserves its mode. Everything here is a build output, so leave nothing unwritable:
    # otherwise a later export or `scp` into the same directory cannot replace it. Only the
    # user-write bit is added, so an executable stays executable.
    for f in readdir(dir; join = true)
        isfile(f) || continue
        m = filemode(f)
        (m & 0o200) == 0 && chmod(f, m | 0o200)
    end
    files = sort!(filter(f -> isfile(joinpath(dir, f)), readdir(dir)))
    return (; dir, mangled, files, r.gains, r.auto)
end

"""
    emit_hardware_harness(dir; Ts, Tf, mangled, gains, auto, log, arm_deg=0.0, card_options=nothing)

Write `run_hardware_config.h` into `dir` and copy `csrc/run_hardware.c` and `csrc/Makefile`
alongside it, next to the exported `top.c`/`top.h` and the C the node calls into.

The control loop itself is a checked-in C file, not a string built here: only the values that
depend on the compiled program are generated, as a small header the loop includes. Those are
the mangled node symbols (`_step`/`_reset`/`_mem`/`_out`), the sample time, the run duration,
the arm's start-up angle, the card options, the log's file name, header and column count, and
the parameter blocks. Keeping the C in a real file means it gets editor tooling and cannot
acquire escaping bugs from being embedded in Julia.

The loop is only timing: `qube_hw_open(QUBE_HW_MODE_HIL, ...)` (home the arm and let the
pendulum hang before starting), `qube_log_open` for the program to write its rows into, then
every `Ts` seconds a single `_step` call, which reads the encoders, computes, writes the motor
and logs the row itself. The `gains`/`auto` parameter objects are serialized to raw bytes and
embedded so `_step` sees the exact byte layout it was compiled against.

Note what is *not* here any more: the loop does not read the node's output struct at all, so
it needs no knowledge of the program's outputs and serves any program built for this rig.
"""
function emit_hardware_harness(dir; Ts, Tf, mangled, gains, auto, log::ProgramLog,
                               arm_deg = 0.0, card_options = nothing)
    words(obj) = join(("0x" * string(w; base = 16, pad = 16) * "ULL" for w in _param_words(obj)), ", ")
    config = """
    /* Auto-generated by QuanserComponents.emit_hardware_harness — do not edit by hand.
     * Everything here depends on the compiled program; run_hardware.c is a normal
     * checked-in C file that includes this. */
    #ifndef RUN_HARDWARE_CONFIG_H
    #define RUN_HARDWARE_CONFIG_H

    #include <stdint.h>

    /* Mangled entry points of the generated node. */
    #define QUBE_STEP  $(mangled)_step
    #define QUBE_RESET $(mangled)_reset
    #define QUBE_MEM   $(mangled)_mem
    #define QUBE_OUT   $(mangled)_out

    #define QUBE_TS $(Float64(Ts))            /* sample time [s]  */
    #define QUBE_TF $(Float64(Tf))            /* run duration [s] */
    /* Where the arm physically sits at start-up; added to every shoulder reading so the
     * arm need not be moved to its home position first. See qube_hw.h. */
    #define QUBE_ARM0 $(deg2rad(Float64(arm_deg)))  /* [rad] */
    $(card_options === nothing ? "/* card options left at the qube_hw.c default */" :
      "#define QUBE_CARD_OPTIONS \"$(card_options)\"")

    /* The log the program writes from inside the tick: the same file name, header and
     * column count the model's DataLogger was built with, so the row the node writes and
     * the file the loop opens cannot disagree. Relative, i.e. in the working directory
     * the loop is started from. */
    #define QUBE_LOG_FILE   "$(log.file)"
    #define QUBE_LOG_HEADER "$(replace(header(log), "\t" => "\\t"))"
    #define QUBE_LOG_NCOLS  $(ncols(log))

    /* Parameter blocks that the generated `_step` reads as opaque pointers: the raw bytes
     * of the Julia TuningGains (the runtime-settable parameters) and AutoPars (all other
     * model constants), serialized so the byte offsets match top.c exactly. */
    static const uint64_t gains_words[] = { $(words(gains)) };
    static const uint64_t auto_words[]  = { $(words(auto)) };
    #define GAINS_PTR ((int64_t)(intptr_t)gains_words)
    #define AUTO_PTR  ((int64_t)(intptr_t)auto_words)

    #endif /* RUN_HARDWARE_CONFIG_H */
    """
    write(joinpath(dir, "run_hardware_config.h"), config)
    for f in ("run_hardware.c", "Makefile")
        cp(joinpath(dirname(QUBE_HW_SRC), f), joinpath(dir, f); force = true)
    end
    return (; c = "run_hardware.c", config = "run_hardware_config.h", makefile = "Makefile")
end

"""
    compile_hardware_harness(dir; quanser_dir="/opt/quanser/hil_sdk") -> exe_path

Compile the emitted `run_hardware.c` + `top.c` + `qube_hw.c` + `qube_log.c` in `dir` into a
`run_hardware` executable, linking the Quanser HIL SDK. Uses the system C compiler (`cc`,
falling back to `gcc`). Linking uses the static SDK libraries, so this does not require the
hardware to be connected — it doubles as a build check for the generated sources, including
that the node's `extern qube_hw_*` and `extern qube_log_row` declarations resolve.
"""
function compile_hardware_harness(dir; quanser_dir = QUANSER_HIL_DIR)
    @info "Compiling hardware harness"
    cc = Sys.which("cc")
    cc === nothing && (cc = Sys.which("gcc"))
    cc === nothing && error("compile_hardware_harness: no C compiler (cc/gcc) found on PATH")
    sdk = quanser_sdk_flags(; quanser_dir)
    sdk.found || error("""
        compile_hardware_harness: no Quanser HIL SDK found (looked for
        $(joinpath(quanser_dir, "include", "hil.h")) and /usr/include/quanser/hil.h).""")
    exe = abspath(joinpath(dir, "run_hardware"))
    args = [cc, "-I$dir", sdk.cflags..., "-O2", "-Wall", "-DQUBE_HW_HAVE_HIL",
            joinpath(dir, "run_hardware.c"), joinpath(dir, "top.c"),
            joinpath(dir, "qube_hw.c"), joinpath(dir, "qube_log.c"), "-o", exe,
            sdk.ldflags..., QUANSER_LIBS...]
    run(Cmd(args))
    return exe
end

"""
    run_hardware_harness(exe) -> log_path

Execute the compiled `run_hardware` binary, blocking for its baked-in run duration `Tf`. The
binary controls the physical pendulum and the program inside it writes the log in its working
directory; that path is returned.
"""
function run_hardware_harness(exe; log_name = SWINGUP_LOG_FILE)
    @info "Running hardware harness"
    exe = abspath(exe)
    dir = dirname(exe)
    # Run with the child's working directory set to `dir` (not the current process's) so the
    # binary — addressed by absolute path — is found and the log lands in `dir`.
    run(Cmd(`$exe`; dir))
    return joinpath(dir, log_name)
end

# ---------------------------------------------------------------------------
## Live plotting
# ---------------------------------------------------------------------------
"""
    launch_live_plot(dir; cmd="kst2", config="kst2config.kst", log=SWINGUP_LOG_FILE, wait_for_log=5.0) -> Process | Nothing

Start a live plotter on the log a run is writing, for watching it as it happens. Returns the
running process, or `nothing` if it could not be started.

`cmd` is the viewer executable and `config` its session file (a relative path resolves
against `dir`); the default pair is [kst2](https://kst-plot.kde.org/) reading the log that
the program appends to. The viewer is launched with its working directory set to `dir`, so a
session file referring to the log by relative path works wherever `dir` is.

Call this *after* starting the run but *before* waiting on it — the plotter has to come up
alongside the run, not after it. It waits up to `wait_for_log` seconds for the log file to
appear first, since a viewer pointed at a missing file typically gives up rather than
retrying.

This never throws: a missing viewer, a missing session file or a failed launch is reported as
a warning and returns `nothing`, because losing the plot is not a reason to abandon a
hardware run. The process outlives this call and is not reaped — `kill` it when done.
"""
function launch_live_plot(dir; cmd = "kst2", config = "kst2config.kst",
                          log = SWINGUP_LOG_FILE, wait_for_log = 5.0)
    dir = abspath(dir)
    cfg = isabspath(config) ? config : joinpath(dir, config)
    if Sys.which(cmd) === nothing
        @warn "live plot: `$cmd` not found on PATH, skipping"
        return nothing
    end
    if !isfile(cfg)
        @warn "live plot: session file not found, skipping" config = cfg
        return nothing
    end
    # A viewer left over from an earlier run is the most confusing failure here: it holds the
    # previous log open, never updates again, and is indistinguishable from the new window.
    # Retire it rather than adding to the pile.
    _close_stale_plotters(cmd, cfg)
    csv = isabspath(log) ? log : joinpath(dir, log)
    t0 = time()
    while !isfile(csv) && time() - t0 < wait_for_log
        sleep(0.05)
    end
    isfile(csv) ||
        @warn "live plot: $(basename(csv)) has not appeared, starting $cmd anyway" wait_for_log
    _check_plot_fields(cfg, csv)
    try
        return run(Cmd(`$cmd $cfg`; dir); wait = false)
    catch e
        @warn "live plot: could not start `$cmd`" exception = e
        return nothing
    end
end

# Terminate viewers already running on this session file. Matches on the config path too, so
# unrelated instances of the same program are left alone.
function _close_stale_plotters(cmd, cfg)
    try
        for line in eachsplit(read(`pgrep -af $(basename(cmd))`, String), '\n')
            occursin(cfg, line) || continue
            pid = tryparse(Int, first(split(line)))
            (pid === nothing || pid == getpid()) && continue
            @info "live plot: retiring a viewer left over from an earlier run" pid
            run(`kill $pid`; wait = false)
        end
    catch
        # pgrep missing, or nothing matched: nothing to clean up.
    end
    return
end

# A kst session names the columns it plots, and names them *as the data source reports them*
# — for an ASCII source with a header, that is the header text. Get one wrong and kst draws an
# empty plot in silence: the vector simply resolves to nothing, and because the x vector is
# shared by every curve, one stale name empties the whole window. That is a genuinely hard
# failure to read at the GUI, so say it here instead.
#
# Session files accumulate vectors as they are edited, and stale ones referring to columns
# that no longer exist are normal and harmless, so a missing field is only worth reporting
# when *nothing* matches — that is the case that means a blank window.
function _check_plot_fields(config, csv)
    (isfile(config) && isfile(csv)) || return
    header = split(strip(first(eachline(csv))), '\t')
    fields = Set(m.captures[1] for m in eachmatch(r"field=\"([^\"]*)\"", read(config, String)))
    delete!(fields, "INDEX")                    # kst's built-in sample index
    isempty(fields) && return
    matched = intersect(fields, Set(header))
    isempty(matched) && @warn """
        live plot: none of the columns $(basename(config)) plots exist in $(basename(csv)),
        so the plot will come up empty. kst names ASCII fields from the header line, so the
        session must use those names.""" session_fields = sort(collect(fields)) log_header = header
    return
end

# Give a viewer something to open before the loop starts writing. `open_log!` truncates in
# place, keeping the inode, so the viewer follows the real run afterwards — whereas a viewer
# started on a missing file gives up, and one started on a file that is later *replaced*
# follows the unlinked one forever.
function _touch_log(log::ProgramLog)
    try
        mkpath(dirname(abspath(log.file)))
        open(log.file, "w") do io
            println(io, header(log))
        end
    catch e
        @warn "could not pre-create the log for the live plot" file = log.file exception = e
    end
    return log.file
end

# ---------------------------------------------------------------------------
## Deployment
# ---------------------------------------------------------------------------
"""
    deploy_hardware_harness(dir; host, remote_dir="furuta_c", ssh=`ssh`, scp=`scp`) -> remote_dir

Copy the exported sources to `host` and build them there. Returns `remote_dir`.

Building on the target rather than cross-compiling is deliberate: the Quanser SDK is
installed on the target (headers, libraries and the board driver plugin for its
architecture), so a cross-toolchain here would have to reproduce that sysroot to gain
nothing. The emitted `Makefile` detects the SDK layout, so the same sources build on an x86
host with the older HIL SDK and on a Raspberry Pi with the `quanser-sdk` packages.

`host` is anything ssh accepts, e.g. `"fredrikb@192.168.1.49"`. Requires non-interactive ssh
(key-based auth) since nothing here can answer a prompt.
"""
function deploy_hardware_harness(dir; host, remote_dir = "furuta_c",
                                 ssh = `ssh`, scp = `scp`)
    @info "Deploying hardware harness"
    missing_files = [f for f in HARNESS_FILES if !isfile(joinpath(dir, f))]
    isempty(missing_files) ||
        error("deploy_hardware_harness: $dir is missing $(join(missing_files, ", ")) — \
               run the export first")
    @info "Creating remote dir"
    # Clear the destinations too. An earlier deploy may have left `synchjulia.h` mode 444 (it
    # originates in the read-only package depot), and `scp` cannot reopen such a file for
    # writing — it fails mid-transfer with "Permission denied".
    remote_files = join(("$remote_dir/$f" for f in HARNESS_FILES), " ")
    run(`$ssh $host "mkdir -p $remote_dir && rm -f $remote_files"`)
    @info "Copying files (scp)"
    run(`$scp $([joinpath(dir, f) for f in HARNESS_FILES]) $host:$remote_dir/`)
    @info "make on remote host"
    run(`$ssh $host "cd $remote_dir && make"`)
    return remote_dir
end

"""
    run_hardware_harness_remote(host, remote_dir; local_dir, log_name, stream_log=false, ssh=`ssh`, scp=`scp`) -> log_path

Run the harness on `host`, blocking for its baked-in duration, then copy the log back into
`local_dir` and return the local path.

With `stream_log`, the remote log is followed over ssh into `local_dir` *while the run is in
progress*, so a live plotter watching the local file sees the run as it happens rather than
only the copy made afterwards.
"""
function run_hardware_harness_remote(host, remote_dir; local_dir,
                                     log_name = SWINGUP_LOG_FILE, stream_log = false,
                                     ssh = `ssh`, scp = `scp`)
    @info "Running hardware harness on remote"
    csv = joinpath(local_dir, log_name)
    remote_csv = "$remote_dir/$log_name"
    # Start from a clean slate so a stale log cannot be mistaken for this run's.
    run(`$ssh $host rm -f $remote_csv`)
    tail = nothing
    if stream_log
        # Truncate in place rather than `rm`. Unlinking would give the new run a fresh inode,
        # and any viewer already watching the old one keeps reading the unlinked file forever
        # -- a live plot that is frozen and looks exactly like a working one. `open(csv, "w")`
        # below truncates, which is all that is wanted.
        # -F retries until the harness creates the file, so this can start first.
        tail = try
            open(csv, "w") do io
                # `stdbuf -oL`: `tail` block-buffers into a pipe by default, which would
                # re-introduce the ~4 KiB granularity the program avoids by line-buffering
                # its own log. Falls back to plain `tail` where stdbuf is absent.
                run(pipeline(`$ssh $host "command -v stdbuf >/dev/null && exec stdbuf -oL tail -F -n +1 $remote_csv || exec tail -F -n +1 $remote_csv"`;
                             stdout = io, stderr = devnull); wait = false)
            end
        catch e
            @warn "could not start log streaming; the log will arrive after the run" exception = e
            nothing
        end
    end
    try
        run(`$ssh $host "cd $remote_dir && ./run_hardware"`)
    finally
        tail === nothing || kill(tail)
    end
    # Copy the finished log back *through* the existing file rather than over it: `scp` would
    # create a new inode and freeze any viewer watching this path, right at the moment the
    # interesting part of the run finished. When the log was streamed this is only a
    # consistency check anyway -- the local copy is already complete.
    tmp = csv * ".fetch"
    run(`$scp $host:$remote_csv $tmp`)
    open(csv, "w") do io
        write(io, read(tmp))
    end
    rm(tmp; force = true)
    return csv
end

# ---------------------------------------------------------------------------
## The four targets
# ---------------------------------------------------------------------------
"""
    HardwareRun

What came of putting a program on the rig: whether it `ran` and on which `target`
(`:none`, `:inprocess`, `:local_c` or `:remote_c`), the local path to the `log` it wrote,
how many `rows` are in it against how many `ticks` were issued, the achieved period as
`(; median_dt, max_dt)`, the `output_dir` and `files` of an export, the `mangled` node
symbol, and the live-plot `plotter` process if one was started.

`ticks` is measured for an in-process run and the intended count (`Tf / Ts`) for an exported
one, whose loop does not report back; `rows` is always what is in the log. They differ when a
run was cut short. The timing comes from the loop for an in-process run and from the log's own
`dt` column otherwise — the program records the period it achieved, so a fetched log carries
it.

`plotter` deliberately outlives the analysis so the trace stays on screen afterwards;
`kill(run.plotter)` closes it.
"""
struct HardwareRun
    ran::Bool
    target::Symbol
    log::Union{Nothing, String}
    rows::Int
    ticks::Int
    timing::NamedTuple{(:median_dt, :max_dt), Tuple{Float64, Float64}}
    output_dir::Union{Nothing, String}
    files::Vector{String}
    mangled::String
    plotter::Union{Nothing, Base.Process}
end

"""
    run_on_target(gen, names; run, export_c, backend, output_dir, Tf, arm_deg,
                  card_options, deploy_host, deploy_dir, live_plot, live_plot_cmd,
                  live_plot_config, gains) -> HardwareRun

Put a compiled program on hardware the way the analysis parameters ask for, and report what
happened. This is the shared body of both analyses' `run_analysis`; see
`QubeHardwareRunBase` (dyad/qube_hardware_run.dyad) for the parameters, which are the same
here.

`names` are the field names for the program's outputs, needed only by the in-process runtime.
`gains` overrides the runtime-settable parameters. Anything that has to happen before the
device is touched (designing a gain) or after the log exists (fitting a model) belongs to the
caller, not here.
"""
function run_on_target(gen, names::Tuple{Vararg{Symbol}}; run::Bool = false,
                       export_c::Bool = false, backend::Symbol = :julia,
                       output_dir = "furuta_c", Tf, arm_deg = 0.0,
                       card_options::Union{Nothing, AbstractString} = nothing,
                       deploy_host::AbstractString = "", deploy_dir = "furuta_c",
                       live_plot::Bool = false, live_plot_cmd = "kst2",
                       live_plot_config = "kst2config.kst", gains = (;),
                       mode::Symbol = :hil)
    log_name = basename(gen.log.file)
    no_timing = (; median_dt = NaN, max_dt = NaN)
    start_plot(dir; log) = live_plot ? launch_live_plot(dir; cmd = live_plot_cmd,
                                                        config = live_plot_config, log) : nothing

    if export_c
        res = export_program_c(gen, output_dir; Tf, arm_deg, card_options, gains)
        run || return HardwareRun(false, :none, nothing, 0, 0, no_timing, output_dir,
                                  res.files, res.mangled, nothing)
        expected = round(Int, Tf / gen.Ts)
        if !isempty(deploy_host)
            # Build and run on another machine (e.g. a Raspberry Pi with the QUBE attached).
            # The log is streamed back during the run when a live plot is wanted, so the
            # viewer watches this run rather than the copy fetched afterwards.
            deploy_hardware_harness(output_dir; host = deploy_host, remote_dir = deploy_dir)
            task = @async run_hardware_harness_remote(deploy_host, deploy_dir;
                                                      local_dir = output_dir, log_name,
                                                      stream_log = live_plot)
            plotter = start_plot(output_dir; log = log_name)
            log = fetch(task)
            return HardwareRun(true, :remote_c, log, _log_rows(log), expected,
                               log_timing(log), output_dir, res.files, res.mangled, plotter)
        end
        exe = compile_hardware_harness(output_dir)
        # With `live_plot` the harness runs on a task so the viewer can be brought up while
        # the run is in progress rather than after it. `fetch` rethrows whatever the harness
        # threw, so failures still surface.
        task = @async run_hardware_harness(exe; log_name)
        plotter = start_plot(output_dir; log = log_name)
        log = fetch(task)
        return HardwareRun(true, :local_c, log, _log_rows(log), expected, log_timing(log),
                           output_dir, res.files, res.mangled, plotter)
    end

    run || return HardwareRun(false, :none, nothing, 0, 0, no_timing, nothing, String[],
                              "", nothing)
    ctrl = make_runtime(gen, names; backend, gains)
    # The loop below does not yield, so the viewer has to be up before it starts. It needs a
    # file to open, hence the header-only stand-in that `open_log!` then truncates in place.
    plotter = nothing
    if live_plot
        _touch_log(gen.log)
        plotter = start_plot(dirname(abspath(gen.log.file)); log = abspath(gen.log.file))
    end
    r = run_program!(ctrl; Tf, arm_deg, card_options, mode)
    return HardwareRun(true, :inprocess, r.log_file, r.rows, r.ticks, r.timing, nothing,
                       String[], "", plotter)
end

# ---------------------------------------------------------------------------
## Shared analysis plumbing
# ---------------------------------------------------------------------------
"""
    program_log_path(spec, default) -> String

Where this run's log goes, given an analysis' `log_file`/`output_dir`/`export_c`.

An exported program runs in its own directory, on whatever machine it was deployed to, so it
gets the bare file name — an absolute path from here would be meaningless there, and the
fetched copy lands in `output_dir` anyway. An in-process run resolves a bare name against
`output_dir`, so its log ends up beside everything else the analysis produced (and beside the
live-plot session file), while a path the caller spelled out is left alone.
"""
function program_log_path(spec, default)
    name = isempty(spec.log_file) ? default : spec.log_file
    spec.export_c && return basename(name)
    return (isabspath(name) || !isempty(dirname(name))) ? name :
           joinpath(spec.output_dir, name)
end

"The backend an analysis asked for, as a `Symbol`. Errors on anything but `julia` or `c`."
function program_backend(spec)
    b = Symbol(spec.backend)
    b in (:julia, :c) ||
        throw(ArgumentError("backend must be \"julia\" or \"c\", got $(repr(spec.backend))"))
    return b
end

"""
    run_trace(r::HardwareRun) -> NamedTuple of vectors

The run's log as a Tables.jl column table, keyed by whatever columns the program logged.
Errors when there is no log to read, which is the honest answer to asking a run that did not
happen for its data.
"""
function run_trace(r::HardwareRun)
    (r.ran && r.log !== nothing && isfile(r.log)) ||
        throw(ArgumentError("No run log available (run the analysis with `run = true`)"))
    return read_log(r.log)
end

# The lines about the run itself that both solutions' `show` want.
function show_run(io::IO, r::HardwareRun)
    r.ran || return
    println(io, "ran on: ", r.target === :remote_c ? "the deploy host (exported C)" :
                            r.target === :local_c ? "this machine (exported C)" :
                            "this process")
    println(io, "log: ", r.log === nothing ? "(none)" : r.log,
            "  (", r.rows, " rows", r.ticks > 0 ? " of $(r.ticks) ticks" : "", ")")
    isnan(r.timing.median_dt) ||
        @printf(io, "achieved period: median %.4f s, max %.4f s\n", r.timing.median_dt,
                r.timing.max_dt)
    r.plotter === nothing || println(io, "live plot: still open (kill it to close)")
    return
end

# Rows of data in a log, without parsing the numbers: the count is all that is wanted here.
function _log_rows(path)
    (path === nothing || !isfile(path)) && return 0
    n = 0
    for (i, l) in enumerate(eachline(path))
        i == 1 && continue                    # header
        isempty(strip(l)) || (n += 1)
    end
    return n
end
