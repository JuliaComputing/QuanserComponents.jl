# Implementation of the FurutaIdentificationExperiment analysis: replay a designed input
# sequence open-loop on the QUBE and hand back what the device did.
#
# `FurutaIdentificationExperiment` is a concrete `analysis` in dyad/identification_experiment.dyad;
# the Dyad compiler generates its spec/entry point, which forwards to
# `FurutaIdentificationBaseSpec` (defined in analysis_base.jl — see the file header there for why
# the forwarding goes through `QubeHardwareRunBaseSpec`). This file provides
# `run_analysis(::FurutaIdentificationBaseSpec)`, which is thin by design: compile the program,
# hand it to [`run_on_target`](@ref), and report. There is no fitting here — the point of the
# experiment is the log, which examples/analyze_discrimination.jl consumes.

export FurutaIdentificationSolution

"""
    FurutaIdentificationSolution

Result of the `FurutaIdentificationExperiment` analysis: `hwrun`, the [`HardwareRun`](@ref)
describing what became of the program, the `log_file` the replay wrote, the `trajectory` it
replayed and how many `nsamples` that held, and the parsed `data`.

`tripped` is `true` when the safety supervisor latched during the run, i.e. the arm passed
`abort_deg` and the rest of the run was commanded 0 V. The replay is open-loop, so that is the
single most important thing to know about a run: the data after the trip describes a coasting
arm, not the designed experiment.
"""
struct FurutaIdentificationSolution{SP <: AbstractQubeHardwareRunBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    hwrun::HardwareRun
    log_file::String
    trajectory::String
    nsamples::Int
    data::Union{Nothing, NamedTuple}
    tripped::Bool
end

function DyadInterface.run_analysis(spec::FurutaIdentificationBaseSpec)
    backend = program_backend(spec)
    mkpath(spec.output_dir)
    log_file = program_log_path(spec, IDENTIFICATION_LOG_FILE)
    traj = identification_traj(spec.traj_file, spec.traj_column)
    # The designed sequence is one sample per tick, so the file's length *is* the run's
    # duration. Stating `Tf` separately would be stating it twice, and the two would drift.
    nsamples = open_traj!(traj)
    Tf = spec.Tf > 0 ? spec.Tf : nsamples * spec.Ts
    # The model's parameters come from `spec.overrides`, which the Dyad compiler builds out of
    # the analysis' `model = FurutaIdentification(final umax = umax, ...)` line. `Ts` and the two
    # file names are structural, so they are not parameters of the built system and are passed to
    # the constructor; the safety angles are converted from degrees here for the same reason they
    # are stated in degrees — that is how the arm's limits are quoted in the manual.
    gen = generate_identification_controller(; spec.Ts, traj_file = spec.traj_file,
                                              traj_column = spec.traj_column, log_file,
                                              warn = deg2rad(spec.warn_deg),
                                              abort = deg2rad(spec.abort_deg),
                                              param_overrides = spec.overrides)
    spec.run && @info "Replaying the designed input" nsamples duration_s = Tf
    hwrun = run_on_target(gen, IDENTIFICATION_OUTPUT_NAMES; spec.run, spec.export_c, backend,
                          spec.output_dir, Tf, spec.arm_deg,
                          card_options = isempty(spec.card_options) ? nothing :
                                         spec.card_options,
                          spec.deploy_host, spec.deploy_dir, spec.live_plot,
                          spec.live_plot_cmd, spec.live_plot_config)

    # Read back whatever log is there, so `run = false` inspects an earlier run without the
    # hardware. A deployed run leaves the fetched copy in `output_dir`, which is `hwrun.log`.
    path = something(hwrun.log, local_log_path(spec, IDENTIFICATION_LOG_FILE))
    data = isfile(path) ? read_log(path) : nothing
    # The supervisor logs its own latch, so this is read rather than inferred.
    tripped = data !== nothing && haskey(data, :tripped) && any(>(0.5), data.tripped)
    sol = FurutaIdentificationSolution(spec, hwrun, path, spec.traj_file, nsamples, data,
                                       tripped)
    tripped && @warn """
        the safety supervisor took over during this run: the arm passed abort_deg = \
        $(spec.abort_deg), so the command was forced to 0 and the tail of the log is a \
        coasting arm rather than the designed experiment."""
    return sol
end

function DyadInterface.AnalysisSolutionMetadata(sol::FurutaIdentificationSolution)
    arts = ArtifactMetadata[]
    if sol.hwrun.output_dir !== nothing
        push!(arts, ArtifactMetadata(:GeneratedFiles, ArtifactType.DataFrame,
            "Generated C files",
            "The SynchToolkit-generated C sources for the replay, with the exported \
             step/reset symbol names."))
    end
    if sol.data !== nothing
        push!(arts, ArtifactMetadata(:Trace, ArtifactType.DataFrame,
            "Replay trace",
            "What the program logged from inside each tick: time [s], the shoulder and elbow \
             angles [rad], the voltage actually applied [V], the designed voltage before the \
             safety supervisor saw it [V], the supervisor's latch, and the loop diagnostics dt \
             and exec [s]. The \
             first four columns are the layout the fitting scripts read."))
    end
    AnalysisSolutionMetadata(arts, Symbol[])
end

function DyadInterface.artifacts(sol::FurutaIdentificationSolution, name::Symbol)
    if name === :GeneratedFiles
        dir = sol.hwrun.output_dir
        dir === nothing &&
            throw(ArgumentError("Nothing was exported (run the analysis with `export_c = true`)"))
        files = sol.hwrun.files
        return (; file = files, bytes = [filesize(joinpath(dir, f)) for f in files])
    elseif name === :Trace
        sol.data === nothing &&
            throw(ArgumentError("No trace available (run the analysis with `run = true`)"))
        return sol.data
    else
        throw(ArgumentError("Unknown artifact `$name`"))
    end
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaIdentificationSolution)
    print(io, "FurutaIdentificationExperiment solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    println(io, "trajectory: ", sol.trajectory, " (", sol.nsamples, " samples, ",
            round(sol.nsamples * sol.spec.Ts, digits = 2), " s)")
    println(io, "log file: ", sol.log_file)
    if sol.hwrun.output_dir !== nothing
        println(io, "output_dir: ", sol.hwrun.output_dir)
    end
    show_run(io, sol.hwrun)
    if sol.tripped
        printstyled(io, "safety supervisor tripped: the tail of this run is a coasting arm\n",
                    color = :yellow, bold = true)
    elseif sol.data !== nothing
        println(io, "arm stayed within bounds; the whole run is the designed experiment")
    end
    if sol.data !== nothing
        println(io, "arm range: ",
                round(rad2deg(minimum(sol.data.shoulder_angle)), digits = 1), " to ",
                round(rad2deg(maximum(sol.data.shoulder_angle)), digits = 1), " deg")
    end
end
