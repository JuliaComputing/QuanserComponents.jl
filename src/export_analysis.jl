# Implementation of the FurutaExportC analysis: build the swing-up controller, export it as C,
# and optionally run it on the QUBE.
#
# `FurutaExportC` is a concrete `analysis` in dyad/furuta_export_c.dyad; the Dyad compiler
# generates its spec/entry point (generated/FurutaExportC_definition.jl), which forwards to
# `FurutaExportCBaseSpec` (defined in analysis_base.jl — see the file header there for why the
# forwarding goes through `QubeHardwareRunBaseSpec`). This file provides
# `run_analysis(::FurutaExportCBaseSpec)`: design the LQR feedback gain from the penalty
# weights `Q1`/`Q2` via `design_lqr`, compile the program, and hand it to
# [`run_on_target`](@ref), which does the exporting, deploying, running and live plotting that
# both analyses share.

using DyadInterface: AbstractAnalysisSolution, ArtifactMetadata,
                     ArtifactType, AnalysisSolutionMetadata

export FurutaExportCSolution

"""
    FurutaExportCSolution

Result of the `FurutaExportC` analysis: the designed LQR gain `L` and `hwrun`, the
[`HardwareRun`](@ref) describing what became of the program — the `output_dir` written to, the
`files` generated, the `mangled` base name of the exported `<mangled>_step`/`_reset` functions,
and, when the analysis ran it, the log it wrote and how the loop kept time.

`L` is the gain the program was built with, whether it was designed here or taken from the
model's tuned default.

`hwrun.plotter` holds the live-plot viewer process when the analysis was run with
`live_plot = true`. It deliberately outlives the analysis so the trace stays on screen
afterwards; `kill(sol.hwrun.plotter)` closes it.
"""
struct FurutaExportCSolution{SP <: AbstractQubeHardwareRunBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    hwrun::HardwareRun
    L::Vector{Float64}
end

function DyadInterface.run_analysis(spec::FurutaExportCBaseSpec)
    backend = program_backend(spec)
    mkpath(spec.output_dir)
    # `design_lqr` costs a couple of minutes, so it stays switched off until asked for; the
    # controller then keeps the tuned gain baked into the model.
    L = nothing # design_lqr(; Ts = spec.Ts, Q1 = spec.Q1, Q2 = spec.Q2)
    # The model's parameters come from `spec.overrides`, which is what the Dyad compiler builds
    # out of the analysis' `model = FurutaHardware(final umax = umax)` line — the analysis'
    # parameters parameterize the model, rather than the implementation reading them back off
    # the spec field by field. `Ts` and `log_file` are the exception: they are structural, so
    # they are not parameters of the built system and have to be passed to the constructor.
    log_file = program_log_path(spec, SWINGUP_LOG_FILE)
    gen = generate_swingup_controller(; spec.Ts, log_file, param_overrides = spec.overrides)
    Tf = spec.Tf > 0 ? spec.Tf : 10.0
    hwrun = run_on_target(gen, SWINGUP_OUTPUT_NAMES; spec.run, spec.export_c, backend,
                          spec.output_dir, Tf, spec.arm_deg,
                          card_options = isempty(spec.card_options) ? nothing :
                                         spec.card_options,
                          spec.deploy_host, spec.deploy_dir, spec.live_plot,
                          spec.live_plot_cmd, spec.live_plot_config, gains = (; L))
    return FurutaExportCSolution(spec, hwrun,
                                 collect(float.(something(L, gen.tuning_defaults[:L]))))
end

function DyadInterface.AnalysisSolutionMetadata(sol::FurutaExportCSolution)
    arts = ArtifactMetadata[]
    if sol.hwrun.output_dir !== nothing
        push!(arts, ArtifactMetadata(:GeneratedFiles, ArtifactType.DataFrame,
            "Generated C files",
            "The SynchToolkit-generated C sources for the swing-up controller, with the \
             exported step/reset symbol names."))
    end
    if sol.hwrun.ran
        push!(arts, ArtifactMetadata(:RunLog, ArtifactType.DataFrame,
            "Hardware run log",
            "Time series the program logged while controlling the hardware: time [s], \
             shoulder/elbow angles [rad], the commanded control voltage [V], the timing \
             diagnostics dt (achieved period [s]) and exec (read-to-write duration [s]), and \
             the raw encoder counts."))
    end
    AnalysisSolutionMetadata(arts, Symbol[])
end

# `:GeneratedFiles` returns a column table (Tables.jl-compatible NamedTuple of vectors) listing
# each generated file, its size in bytes, and the exported C symbol on the source/header rows.
# `:RunLog` (only when the analysis ran) returns the hardware trace, parsed from the log with
# the same reader every other log in this package goes through.
function DyadInterface.artifacts(sol::FurutaExportCSolution, name::Symbol)
    if name === :GeneratedFiles
        dir = sol.hwrun.output_dir
        dir === nothing &&
            throw(ArgumentError("Nothing was exported (run the analysis with `export_c = true`)"))
        files = sol.hwrun.files
        bytes = [filesize(joinpath(dir, f)) for f in files]
        symbol = map(files) do f
            f == "top.c" ? "$(sol.hwrun.mangled)_step" :
            f == "top.h" ? "$(sol.hwrun.mangled)_reset" : ""
        end
        return (; file = files, bytes = bytes, symbol = symbol)
    elseif name === :RunLog
        return run_trace(sol.hwrun)
    else
        throw(ArgumentError("Unknown artifact `$name`"))
    end
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaExportCSolution)
    print(io, "FurutaExportC solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    if sol.hwrun.output_dir !== nothing
        println(io, "output_dir: ", sol.hwrun.output_dir)
        println(io, "files: ", join(sol.hwrun.files, ", "))
        println(io, "step/reset: ", sol.hwrun.mangled, "_step / ", sol.hwrun.mangled, "_reset")
    end
    println(io, "gain L: ", sol.L)
    show_run(io, sol.hwrun)
end
