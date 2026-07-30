# Implementation of the FurutaFrictionExperiment analysis: run the constant-velocity
# friction experiment on the physical QUBE and fit a friction model to what it logged.
#
# `FurutaFrictionExperiment` is a concrete `analysis` in `dyad/friction_experiment.dyad`;
# the Dyad compiler generates its spec/entry point, whose `run_analysis` forwards to
# `FurutaFrictionBaseSpec` (defined in analysis_base.jl — see the file header there for why the
# forwarding goes through `QubeHardwareRunBaseSpec`). This file is the whole
# analysis: run the experiment, select the usable samples, fit, report, and expose the
# result as artifacts and a plot recipe.
#
# ## Why the fit is what it is
#
# At a genuinely constant velocity the equation of motion
#
#     τ_motor = J ω̇ + τ_f(ω)
#
# loses its inertia term, so the applied torque *is* the friction torque and no inertia has
# to be modelled. Everything therefore hinges on selecting only the samples where the
# acceleration really is negligible — that selection, not the regression, is the part worth
# looking at the plots for, which is why `plot(sol)` shows the selected samples on top of the
# trace and the diagnostic signals against their thresholds.
#
# The regression is linear least squares on
#
#     u(ω) = a₁ sign(ω) + a₂ ω + a₃ sign(ω) ω² + a₄ ω³
#
# in *motor-command* space, because that is what is measured and it is well scaled.
# `FrictionParams` is in torque space, so converting is just the command-to-torque gain:
#
#     τ_f(ω) = kt/Rm ⋅ u(ω)
#
# with **no** back-EMF subtracted. That is deliberate: back-EMF and speed-dependent
# friction both appear as volts per rad/s, so this experiment cannot separate them, and
# subtracting a `km` from elsewhere merely injects the disagreement between the two into
# the answer. The coefficients are the axis' whole speed-dependent torque
# (`FrictionAndBackEMF`), which also makes the result dissipative by construction: `u(ω)`
# rises with speed, so `τ_f(ω)` does too.

using Printf: @printf, @sprintf
using RecipesBase: RecipesBase, plot, @recipe, @series
using LinearAlgebra: norm

export FurutaFrictionSolution, FrictionFit, fit_friction, friction_data, friction_report,
       read_friction_log

"""
    FrictionFit

What the friction fit produced.

  - `params`: the [`FrictionParams`](@ref) set, in torque space — this is the deliverable,
    ready to paste into dyad/definitions.jl (see [`friction_report`](@ref)).
  - `a`: the raw command-space coefficients `[sign(ω), ω, sign(ω)ω², ω³]`, in volts. `a[1]`
    is the breakaway command, directly comparable across experiments without needing the
    motor constants.
  - `resid_rms`: residual RMS of the regression, in volts, against `command_range` — the
    ratio is the honest measure of how well a static friction curve describes the data.
  - `nkeep` of `nsamples`: how much of the log survived the constant-velocity selection.
  - `motor`: the `IdParams` whose `kt` and `Rm` gave the command-to-torque gain.
"""
struct FrictionFit
    params::FrictionParams
    a::Vector{Float64}
    resid_rms::Float64
    command_range::Float64
    nkeep::Int
    nsamples::Int
    w_scale::Float64
    motor::IdParams
end

"""
    FurutaFrictionSolution

Result of the `FurutaFrictionExperiment` analysis: `hwrun` describing the run itself (see
[`HardwareRun`](@ref) — where it ran, the log, how many rows against how many ticks, the
achieved loop period), the `log_file` the fit read, the parsed `data`, and the `fit`.

`hwrun.ran` is `false` when the analysis was run with `run = false`, which compiles the
program and skips the hardware; the fit still happens if a log from an earlier run is there,
so `run = false` is also how you re-fit an existing log with different selection thresholds.

`fit` is `nothing` when there was no log to fit or too little of it survived selection.

`plot(sol)` shows everything: the trace with the selected samples marked, the command, the
selection diagnostics against their thresholds, and the fitted curve over the data.
"""
struct FurutaFrictionSolution{SP <: AbstractQubeHardwareRunBaseSpec} <: AbstractAnalysisSolution
    spec::SP
    hwrun::HardwareRun
    log_file::String
    data::Union{Nothing, NamedTuple}
    fit::Union{Nothing, FrictionFit}
end

# The run's own properties are the `HardwareRun`'s, not copies of them.
ran(sol::FurutaFrictionSolution) = sol.hwrun.ran

function DyadInterface.run_analysis(spec::FurutaFrictionBaseSpec)
    backend = program_backend(spec)
    mkpath(spec.output_dir)
    # One full up-and-down sweep unless asked otherwise. Anything shorter truncates the
    # reference schedule, which costs the fit its highest speeds or one whole direction.
    Tf = spec.Tf > 0 ? spec.Tf :
         friction_sweep_duration(; n_levels = spec.n_levels, t_step = spec.t_step)
    # `log_file` reaches the model, not only the driver: it is the `DataLogger`'s structural
    # `filename`, so the component that writes the file and the call that opens it cannot
    # disagree about which file that is.
    # The model's parameters come from `spec.overrides`, which is what the Dyad compiler
    # builds out of the analysis' `model = FurutaFriction(final K = K, ...)` line — the
    # analysis' parameters parameterize the model, rather than the implementation reading
    # them back off the spec field by field. `Ts` and `log_file` are the exception: they are
    # structural, so they are not parameters of the built system and have to be passed to the
    # constructor.
    log_file = program_log_path(spec, FRICTION_LOG_FILE)
    gen = generate_friction_controller(; spec.Ts, log_file, param_overrides = spec.overrides)
    spec.run && @info "Running friction experiment"
    hwrun = run_on_target(gen, FRICTION_OUTPUT_NAMES; spec.run, spec.export_c, backend,
                          spec.output_dir, Tf, spec.arm_deg,
                          card_options = isempty(spec.card_options) ? nothing :
                                         spec.card_options,
                          spec.deploy_host, spec.deploy_dir, spec.live_plot,
                          spec.live_plot_cmd, spec.live_plot_config)
    spec.run && @info "Experiment done"

    # Fit whatever log is there, so `run = false` re-fits an earlier one with new thresholds
    # instead of needing the hardware again. A deployed run leaves the fetched copy in
    # `output_dir` under the same name, which is what `hwrun.log` points at.
    path = something(hwrun.log, log_file)
    data, fit = nothing, nothing
    if isfile(path)
        data = friction_data(read_log(path); spec.settle, spec.acc_tol, spec.elbow_tol,
                             spec.w_min_fit, spec.w_max_fit, spec.smooth_n)
        @info "Fitting friction model"
        fit = fit_friction(data)
    elseif spec.run
        @warn "the program reported rows but no log file is readable" file = path
    end

    sol = FurutaFrictionSolution(spec, hwrun, path, data, fit)
    # The fitted parameters are the point of running this, so they go to the terminal without
    # being asked for. `show` prints the same report.
    fit === nothing || print(friction_report(fit))
    return sol
end

# ---------------------------------------------------------------------------
## Data selection
# ---------------------------------------------------------------------------
"Central difference, ends held so the result keeps its length."
function _centraldiff(x)
    n = length(x)
    d = similar(x)
    n < 3 && return fill!(d, 0)
    @inbounds for i in 2:(n - 1)
        d[i] = (x[i + 1] - x[i - 1]) / 2
    end
    d[1], d[end] = d[2], d[end - 1]
    return d
end

_mean(v) = isempty(v) ? NaN : sum(v) / length(v)

"Zero-phase moving average of width `n` — forward then backward, as `filtfilt` does."
function _smooth(x, n)
    n <= 1 && return copy(x)
    ma(v) = [_mean(view(v, max(1, i - n + 1):i)) for i in eachindex(v)]
    return reverse(ma(reverse(ma(x))))
end

"""
    friction_data(log; settle, acc_tol, elbow_tol, w_min_fit, w_max_fit, smooth_n)

Add the derived signals the fit needs to a parsed friction log and mark the samples that
are usable, returning the log's columns plus:

  - `acc`: smoothed arm angular acceleration, the constant-velocity test itself.
  - `w_elbow`: pendulum angular velocity — at a constant arm speed the pendulum should be
    at rest too, and while it swings it is a torque disturbance on the axis being measured.
  - `since_step`: time since the velocity reference last changed, so the transient after
    each step can be dropped wholesale rather than relying on `acc` alone.
  - `keep`: the selection mask, true where all four thresholds are satisfied.

`w` is the velocity the *program* estimated (`VelocityEstimator`, already filtered), not a
differentiated angle — the controller acted on that signal, so it is the one the command
corresponds to.
"""
function friction_data(log; settle = 0.6, acc_tol = 3.0, elbow_tol = 1.5,
                       w_min_fit = 0.5, w_max_fit = 40.0, smooth_n = 20)
    t, wref, w = log.time, log.w_ref, log.shoulder_velocity
    Ts = _median(diff(t))
    acc = _smooth(_centraldiff(w) ./ Ts, smooth_n)
    w_elbow = _centraldiff(log.elbow_angle) ./ Ts

    since_step = similar(t)
    last_change, prev = isempty(t) ? (0.0, 0.0) : (t[1], wref[1])
    for i in eachindex(t)
        wref[i] == prev || (last_change = t[i]; prev = wref[i])
        since_step[i] = t[i] - last_change
    end

    keep = (since_step .>= settle) .& (abs.(acc) .< acc_tol) .&
           (abs.(w_elbow) .< elbow_tol) .&
           (abs.(w) .>= w_min_fit) .& (abs.(w) .<= w_max_fit)
    return (; log..., Ts, acc, w_elbow, since_step, keep,
              thresholds = (; settle, acc_tol, elbow_tol, w_min_fit, w_max_fit))
end

# ---------------------------------------------------------------------------
## The fit
# ---------------------------------------------------------------------------
_signsquare(x) = sign(x) * x^2

"""
    fit_friction(data; motor = identified) -> FrictionFit | nothing

Fit the friction model to the samples `data.keep` selected, by least squares in
motor-command space, and convert the result to the torque space `FrictionParams` uses with
`motor`'s `kt` and `Rm` (see this file's header for the algebra).

Returns `nothing`, with a warning, when too little of the log survived selection to fit
four coefficients meaningfully — losing the trace to an exception would be worse.

`motor` defaults to the `identified` parameter set, since that is what the plant these
numbers feed back into uses.
"""
function fit_friction(data; motor::IdParams = identified)
    keep = data.keep
    nkeep, nsamples = count(keep), length(keep)
    if nkeep <= 20
        @warn """
            Only $nkeep of $nsamples samples survived the constant-velocity selection, too
            few to fit. Loosen `settle` / `acc_tol` / `elbow_tol`, or raise `t_step` so each
            speed has time to settle.""" _module = nothing _file = nothing
        return nothing
    end
    w, u = data.shoulder_velocity[keep], data.control_input[keep]
    # Both signs are what separates the Coulomb term from a constant offset in the command.
    (any(>(0), w) && any(<(0), w)) ||
        @warn "only one sign of velocity survived selection; kc is confounded with an offset"

    sc = maximum(abs, w)               # scale for conditioning; sign() is scale-free
    ws = w ./ sc
    a = [sign.(w) ws _signsquare.(ws) ws .^ 3] \ u
    a[2] /= sc
    a[3] /= sc^2
    a[4] /= sc^3

    resid = u .- _command_model(w, a)
    g = motor.kt / motor.Rm            # command [V] -> torque [N·m]
    params = FrictionParams(; kc = g * a[1],
                              kv = g * a[2],        # friction and back-EMF together
                              k2 = g * a[3],
                              k3 = g * a[4],
                              w_tanh = friction_nominal.w_tanh)  # not identifiable here
    fit = FrictionFit(params, a, norm(resid) / sqrt(length(resid)),
                      maximum(u) - minimum(u), nkeep, nsamples, sc, motor)
    params.kc > 0 || @warn """
        kc came out non-positive, which is not physical. The likeliest cause is deadband
        compensation having been left on during the experiment: it adds a fixed offset in
        the direction of the command, straight onto the term being measured. Re-run with
        `card_options = "deadband_compensation=0.0"` (the analysis default)."""
    return fit
end

# The fitted command curve, evaluated with a hard sign (`smooth = false`) or the smoothed
# one the `Friction` component uses by default.
_command_model(w, a) = a[1] .* sign.(w) .+ a[2] .* w .+ a[3] .* _signsquare.(w) .+
                       a[4] .* w .^ 3
function _command_model_smooth(w, a, w_tanh)
    sw = tanh.(w ./ w_tanh)
    return a[1] .* sw .+ a[2] .* w .+ a[3] .* sw .* w .^ 2 .+ a[4] .* w .^ 3
end

# ---------------------------------------------------------------------------
## Reporting
# ---------------------------------------------------------------------------
"""
    friction_report(fit) -> String

The fitted parameters as a printable report, ending in a ready-to-paste `withparams` call
for `friction_identified` in dyad/definitions.jl. Printed automatically when the analysis
runs and by `show`.
"""
function friction_report(fit::FrictionFit)
    io = IOBuffer()
    println(io, "\n================ identified friction ================")
    @printf(io, "fit over %d of %d samples: residual RMS %.4f V of %.2f V command range\n",
            fit.nkeep, fit.nsamples, fit.resid_rms, fit.command_range)
    @printf(io, "breakaway command   a1 = %+.4f V\n", fit.a[1])
    @printf(io, "Coulomb             kc = %+.4e N·m\n", fit.params.kc)
    @printf(io, "first order         kv = %+.4e N·m·s/rad\n", fit.params.kv)
    @printf(io, "quadratic           k2 = %+.4e\n", fit.params.k2)
    @printf(io, "cubic               k3 = %+.4e\n", fit.params.k3)
    println(io, "=====================================================")
    println(io, "\n# paste into dyad/definitions.jl (replaces `friction_identified`):")
    println(io, "const friction_identified = withparams(friction_nominal;")
    for f in (:kc, :kv, :k2, :k3)
        @printf(io, "    %-7s = %.8g,\n", f, getfield(fit.params, f))
    end
    println(io, ")")
    println(io, "# then use it with e.g. FrictionAndBackEMF(params = friction_identified)")
    return String(take!(io))
end

function Base.show(io::IO, ::MIME"text/plain", sol::FurutaFrictionSolution)
    print(io, "FurutaFrictionExperiment solution for ")
    printstyled(io, "$(nameof(sol.spec))\n", color = :green, bold = true)
    println(io, "log file: ", sol.log_file)
    if sol.hwrun.ran
        show_run(io, sol.hwrun)
        sol.hwrun.rows == sol.hwrun.ticks || sol.hwrun.ticks == 0 ||
            println(io, "   (MISMATCH — the log lost rows)")
    else
        println(io, "not run (run = false); the program was compiled only")
    end
    if sol.fit === nothing
        println(io, "no fit", sol.data === nothing ? " (no log to fit)" :
                              " (too few samples survived selection)")
    else
        print(io, friction_report(sol.fit))
        println(io, "\n`plot(sol)` shows the trace, the selection and the fitted curve")
    end
end

# ---------------------------------------------------------------------------
## Artifacts
# ---------------------------------------------------------------------------
function DyadInterface.AnalysisSolutionMetadata(sol::FurutaFrictionSolution)
    arts = ArtifactMetadata[]
    if sol.data !== nothing
        push!(arts, ArtifactMetadata(:ExperimentPlot, ArtifactType.PlotlyPlot,
            "Friction experiment",
            "What the experiment did: the velocity reference against the estimated \
             velocity with the samples the fit used marked, the motor command, and the \
             selection diagnostics (arm acceleration, pendulum velocity) against their \
             thresholds. A selection that keeps transients shows up here."))
        push!(arts, ArtifactMetadata(:Trace, ArtifactType.DataFrame,
            "Friction experiment trace",
            "What the program logged from inside each tick: elapsed time [s], the velocity \
             reference and the estimated shoulder velocity [rad/s], the shoulder and elbow \
             angles [rad], and the applied motor voltage [V]."))
    end
    if sol.fit !== nothing
        push!(arts, ArtifactMetadata(:FitPlot, ArtifactType.PlotlyPlot,
            "Friction model fit",
            "Motor command against velocity for the selected samples, with the fitted \
             curve in both its hard-sign and tanh-smoothed forms. A bad selection shows up \
             as a fan of points rather than a curve."))
        push!(arts, ArtifactMetadata(:FrictionParameters, ArtifactType.DataFrame,
            "Identified friction parameters",
            "The fitted coefficients: the command-space regression coefficients as \
             measured, and the torque-space FrictionParams derived from them. `kv` is \
             directly comparable with the plant's `br`."))
        push!(arts, ArtifactMetadata(:FrictionParams, ArtifactType.Native,
            "FrictionParams object",
            "The identified `FrictionParams`, ready to hand to `Friction(params = ...)`."))
    end
    AnalysisSolutionMetadata(arts, Symbol[])
end

function DyadInterface.artifacts(sol::FurutaFrictionSolution, name::Symbol)
    if name === :ExperimentPlot
        _require(sol.data !== nothing, "No trace available (run the analysis with `run = true`)")
        return plot(sol; friction_panels = :experiment)
    elseif name === :FitPlot
        _require(sol.fit !== nothing, "No fit available")
        return plot(sol; friction_panels = :fit)
    elseif name === :Trace
        _require(sol.data !== nothing, "No trace available (run the analysis with `run = true`)")
        cols = Symbol.(FRICTION_LOG_COLUMNS)
        return NamedTuple{Tuple(cols)}(ntuple(j -> getfield(sol.data, cols[j]), length(cols)))
    elseif name === :FrictionParameters
        _require(sol.fit !== nothing, "No fit available")
        f = sol.fit
        return (; quantity = ["Coulomb", "first order", "quadratic", "cubic"],
                  command = f.a,
                  torque = [f.params.kc, f.params.kv, f.params.k2, f.params.k3],
                  unit = ["N·m", "N·m·s/rad", "N·m·s²/rad²", "N·m·s³/rad³"])
    elseif name === :FrictionParams
        _require(sol.fit !== nothing, "No fit available")
        return sol.fit.params
    else
        throw(ArgumentError("Unknown artifact `$name`"))
    end
end

_require(cond, msg) = cond || throw(ArgumentError(msg))

"""
    read_friction_log(path) -> NamedTuple of vectors

[`read_log`](@ref) under the name this experiment has always used it by. Every log in this
package is written by a `DataLogger` inside the program and read back by that one function,
which takes the column names from the file's header rather than from a copy of the layout kept
here.
"""
read_friction_log(path) = read_log(path)

# ---------------------------------------------------------------------------
## Plotting
# ---------------------------------------------------------------------------
# One recipe covering every panel, so `plot(sol)` needs no arguments and shows the whole
# story: what the experiment did, which samples the fit was allowed to see, why the rest
# were rejected, and the resulting curve over the data. `friction_panels` restricts it,
# which is what the two plot artifacts use.
#
# `RecipesBase` rather than Plots: this stays a light dependency and the recipe resolves
# against whichever backend the caller has loaded (Plotly, in Dyad Builder).
@recipe function f(sol::FurutaFrictionSolution)
    # Read the attribute out of `plotattributes` rather than declaring it as a recipe
    # keyword: `@recipe` compiles a keyword into a call to `RecipesBase.is_key_supported`,
    # which is declared with *no methods* and only gains them from a plotting backend, so a
    # declared keyword makes the recipe depend on which backend happens to be loaded.
    # `pop!` also keeps the attribute from reaching the backend as an unknown one.
    friction_panels = pop!(plotattributes, :friction_panels, :all)
    friction_panels in (:all, :experiment, :fit) ||
        throw(ArgumentError("friction_panels must be :all, :experiment or :fit, got \
                             $(repr(friction_panels))"))
    d = sol.data
    d === nothing && throw(ArgumentError("nothing to plot: the analysis produced no trace"))
    want_fit = friction_panels !== :experiment && sol.fit !== nothing
    want_exp = friction_panels !== :fit

    npanels = (want_exp ? 3 : 0) + (want_fit ? 1 : 0)
    npanels == 0 && throw(ArgumentError("nothing to plot for friction_panels = \
                                         $(repr(friction_panels))"))
    layout --> (npanels, 1)
    # Stacked time series want to be wide and short; the fit on its own is a scatter plot
    # against velocity and wants to be roughly square, or the curve is unreadable.
    size --> (want_fit && !want_exp ? (780, 580) : (900, 280 * npanels))
    framestyle --> :zerolines
    legend --> :outertopright
    panel = 0

    if want_exp
        th = d.thresholds
        # 1. the reference and what the axis did, with the selected samples marked
        panel += 1
        @series begin
            subplot := panel
            label := ["reference" "measured"]
            title := "friction experiment"
            ylabel := "ω [rad/s]"
            d.time, [d.w_ref d.shoulder_velocity]
        end
        @series begin
            subplot := panel
            seriestype := :scatter
            label := "selected ($(count(d.keep)) samples)"
            markersize := 1.2
            markeralpha := 0.4
            markerstrokewidth := 0
            d.time[d.keep], d.shoulder_velocity[d.keep]
        end
        # 2. the command, which is the regression's target
        panel += 1
        @series begin
            subplot := panel
            label := "command"
            ylabel := "u [V]"
            d.time, d.control_input
        end
        # 3. why samples were rejected: the two disturbance signals against their limits
        panel += 1
        @series begin
            subplot := panel
            label := ["arm acceleration" "pendulum velocity"]
            ylabel := "selection diagnostics"
            xlabel := "t [s]"
            d.time, [d.acc d.w_elbow]
        end
        @series begin
            subplot := panel
            seriestype := :hline
            label := ""
            linestyle := :dash
            linecolor := :black
            [-th.acc_tol, th.acc_tol]
        end
    end

    if want_fit
        fit = sol.fit
        w, u = d.shoulder_velocity[d.keep], d.control_input[d.keep]
        wgrid = range(-fit.w_scale, fit.w_scale, length = 601)
        panel += 1
        @series begin
            subplot := panel
            seriestype := :scatter
            label := "selected data"
            markersize := 2
            markeralpha := 0.5
            markerstrokewidth := 0
            title := "friction model"
            xlabel := "ω [rad/s]"
            ylabel := "u [V]"
            w, u
        end
        @series begin
            subplot := panel
            label := "fit (sign)"
            linewidth := 2
            wgrid, _command_model(wgrid, fit.a)
        end
        @series begin
            subplot := panel
            label := "fit (tanh, w_tanh = $(fit.params.w_tanh))"
            linewidth := 2
            linestyle := :dash
            wgrid, _command_model_smooth(wgrid, fit.a, fit.params.w_tanh)
        end
    end
end
