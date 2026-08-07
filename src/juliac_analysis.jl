# Implementation of the FurutaSwingupJuliaC analysis: the swing-up controller deployed as a
# statically compiled Julia binary.
#
# `FurutaSwingupJuliaC` (dyad/swingup_juliac.dyad) extends `FurutaSwingupJuliaCBase`
# (dyad/partial_swingup_juliac.dyad), so it shares the parameter set of every hardware run on
# this rig; the Dyad compiler resolves its spec to the root of that chain and the fan-out in
# analysis_base.jl brings it back here, to `FurutaSwingupJuliaCBaseSpec` and this
# `run_analysis`. What belongs to this file is the target: emitting the application, compiling
# it and running it — see juliac.jl, which does the work.
#
# The sibling of swingup_analysis.jl, which does the same for the in-process and C targets.

export FurutaSwingupJuliaCSolution

"""
    FurutaSwingupJuliaCSolution

Result of the `FurutaSwingupJuliaC` analysis: the designed LQR gain `L`, the emitted
application (`app_dir` and its `files`), the compiled `exe` and the JuliaC `buildlog`
(`nothing` when the analysis ran with `build = false`), and, when it ran the binary, the `log`
the program wrote.

`L` is the gain the program was built with, whether designed here or taken from the model's
tuned default.
"""
struct FurutaSwingupJuliaCSolution{SP <: AbstractQubeHardwareRunBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    app_dir::String
    files::Vector{String}
    L::Vector{Float64}
    exe::Union{Nothing, String}
    buildlog::Union{Nothing, String}
    ran::Bool
    log::Union{Nothing, String}
end

function DyadInterface.run_analysis(spec::FurutaSwingupJuliaCBaseSpec)
    mkpath(spec.output_dir)
    # As in the C analysis: `design_lqr` costs a couple of minutes, so it stays switched off
    # until asked for and the controller keeps the tuned gain baked into the model.
    L = nothing # design_lqr(; Ts = spec.Ts, Q1 = spec.Q1, Q2 = spec.Q2)
    # An exported application runs in its own directory, so the log is a bare file name there;
    # an absolute path from this machine would be meaningless once the binary is moved.
    log_file = basename(program_log_path(spec, SWINGUP_LOG_FILE))
    gen = compile_program_source(FurutaHardware; name = :controller, spec.Ts,
                                 tunables = SWINGUP_TUNABLES, outputs = _swingup_outputs,
                                 log = swingup_log(log_file),
                                 param_overrides = spec.overrides)
    Tf = spec.Tf > 0 ? spec.Tf : 10.0
    r = export_program_juliac(gen, spec.output_dir; spec.app_name, Tf, spec.arm_deg,
                              card_options = isempty(spec.card_options) ? nothing :
                                             spec.card_options,
                              gains = (; L, umax = spec.umax),
                              spec.julia_channel, spec.trim,
                              build = spec.build || spec.run)
    # `run = true` drives the physical pendulum with the compiled binary, which writes the same
    # log any other target would; it implies a build, since there is otherwise nothing to run.
    log = spec.run ? run_program_juliac(r.exe; log_name = log_file) : nothing
    return FurutaSwingupJuliaCSolution(spec, r.app_dir, collect(r.files),
                                       collect(float.(something(L, gen.tuning_defaults[:L]))),
                                       r.exe, r.buildlog, spec.run, log)
end

function DyadInterface.AnalysisSolutionMetadata(sol::FurutaSwingupJuliaCSolution)
    arts = [ArtifactMetadata(:GeneratedFiles, ArtifactType.DataFrame,
        "Generated application",
        "The emitted Julia application package for the swing-up controller — the \
         code-generated node, the operator wrappers it calls into, the application module \
         with the timing loop, and (when built) the compiled standalone binary.")]
    if sol.buildlog !== nothing
        push!(arts, ArtifactMetadata(:BuildLog, ArtifactType.Native,
            "JuliaC build log",
            "Combined output of the environment instantiation and the JuliaC \
             --output-exe --trim build of the application package."))
    end
    if sol.ran
        push!(arts, ArtifactMetadata(:RunLog, ArtifactType.DataFrame,
            "Hardware run log",
            "Time series the program logged while controlling the hardware: time [s], \
             shoulder/elbow angles [rad], the commanded control voltage [V], the timing \
             diagnostics dt (achieved period [s]) and exec (read-to-write duration [s]), and \
             the raw encoder counts."))
    end
    AnalysisSolutionMetadata(arts, Symbol[])
end

# `:GeneratedFiles` returns a column table (Tables.jl-compatible NamedTuple of vectors) of the
# emitted files — path relative to the application, size in bytes, and what each is for — with
# the compiled binary appended when there is one. `:BuildLog` returns JuliaC's output as a
# string, `:RunLog` the hardware trace, read with the same reader every other log goes through.
function DyadInterface.artifacts(sol::FurutaSwingupJuliaCSolution, name::Symbol)
    if name === :GeneratedFiles
        paths = copy(sol.files)
        roles = map(sol.files) do f
            b = basename(f)
            b == "Project.toml" ? "application package" :
            b == "controller.jl" ? "code-generated synchronous node" :
            b == "hardware_ffi.jl" ? "hardware I/O and logging the node calls" :
            b == "README.md" ? "build instructions" :
            endswith(b, ".c") || endswith(b, ".h") ? "C the node calls into" :
            "application (timing loop, @main)"
        end
        bytes = [filesize(joinpath(sol.app_dir, p)) for p in paths]
        if sol.exe !== nothing && isfile(sol.exe)
            push!(paths, relpath(sol.exe, sol.app_dir))
            push!(roles, "compiled standalone binary")
            push!(bytes, filesize(sol.exe))
        end
        return (; file = paths, bytes = bytes, role = roles)
    elseif name === :BuildLog
        (sol.buildlog !== nothing && isfile(sol.buildlog)) ||
            throw(ArgumentError("No build log available (run the analysis with `build = true`)"))
        return read(sol.buildlog, String)
    elseif name === :RunLog
        (sol.ran && sol.log !== nothing && isfile(sol.log)) ||
            throw(ArgumentError("No run log available (run the analysis with `run = true`)"))
        return read_log(sol.log)
    else
        throw(ArgumentError("Unknown artifact `$name`"))
    end
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaSwingupJuliaCSolution)
    print(io, "FurutaSwingupJuliaC solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    println(io, "application: ", sol.app_dir)
    println(io, "files: ", join(sol.files, ", "))
    println(io, "gain L: ", sol.L)
    println(io, "executable: ", sol.exe === nothing ? "(not built)" : sol.exe)
    if sol.ran
        println(io, "run log: ", sol.log === nothing ? "(none)" : sol.log)
    end
end
