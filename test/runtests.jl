using QuanserComponents
using Test
# SynchCompiler ≥ 0.4 provides its C compiler through a package extension; the `:c`
# backend and C export used below require Clang_unified_jll to be loaded.
using Clang_unified_jll
using ModelingToolkit
using SymbolicIndexingInterface: default_values
using MultibodyComponents
# using DiscreteComponents
using SynchToolkit
using LinearAlgebra
using Statistics: cor
using DelimitedFiles: readdlm
using Printf: @sprintf
# The plot-artifact assertions build real `Plots.Plot`s, so a backend has to be loaded.
# (The `using Plots` further down is inside a disabled block and does not count.)
using Plots: Plots
# using SynchJulia
# SynchJulia.backend!(:julia)
##
# include("../generated/tests.jl")


@time "model" @named model = QuanserComponents.FurutaSwingup();
@time "mtkcompile + compile_lustre" ssys = multibody(model, additional_passes=[SynchToolkit.compile_lustre])

#
k = ModelingToolkit.ShiftIndex()
##
@time "ODEProblem" prob = ODEProblem(ssys, [
    ssys.qubependulum.shoulder_joint.render => false
    # ssys.qubependulum.shoulder_joint.radius => 0.01
    # ssys.qubependulum.shoulder_joint.color => [0.8, 0.8, 0.8, 1]
    # ssys.qubependulum.shoulder_joint.cylinder_length => 0.03
    ssys.qubependulum.elbow_joint.phi => 0+deg2rad(0.15)
    ssys.qubependulum.shoulder_joint.phi => 0.0
    ssys.gain.k => 1.0
    # Controller gains (energyswingup gain/arm_centering/umax, lqrstabilizer L/umax)
    # now come from the model defaults.

    ssys.qubependulum.floor.r_shape => [0, -0.10, 0]

    # ssys.qubependulum.base_box.color => [0.1, 0.1, 0.1, 1]
    # ssys.qubependulum.base_box.shape.specular_coefficient => 1.5

    # ssys.qubependulum.base_box.shape_transform => MultibodyComponents.Rp2T(MultibodyComponents.RotXYZ(-pi/2, 0, -pi / 2), [0, -0.075, 0])
    # ssys.qubependulum.motor_front_mesh.shape_transform => MultibodyComponents.Rp2T(MultibodyComponents.RotY(-pi / 2) * MultibodyComponents.RotX(-pi / 2), [0.025, 0, 0])


    # ssys.qubependulum.lower_arm.shape_transform => MultibodyComponents.Rp2T(MultibodyComponents.RotY(pi / 2) * MultibodyComponents.RotX(pi / 2), [0, -0.058, 0])

], (0.0, 10.0))



using OrdinaryDiffEqLowOrderRK

@time "solve" sol = solve(prob, BS3(), dt=0.005)
@assert !all(isnan, sol[ssys.control_system.elbow_angle])



@test abs(mod2pi(sol(sol.t[end], idxs=ssys.qubependulum.elbow_joint.phi)) - pi) < 0.01

if false
    import GLMakie
    using GLMakie: Makie, AmbientLight, SpotLight, PointLight, RGBf, Vec3f, Vec2f
    # Spotlight rig — primary key light from above-front-right with a small cone,
    # soft blue rim from behind, low ambient for contrast.
    qube_lights = [
        AmbientLight(RGBf(0.12, 0.12, 0.12)),
        SpotLight(RGBf(2.2, 2.2, 2.2),
                Vec3f(0.4, 0.6, 0.4),
                Vec3f(0, 0, 0) - Vec3f(0.4, 0.6, 0.4),
                Vec2f(deg2rad(15), deg2rad(35))),
        PointLight(RGBf(0.45, 0.45, 0.6), Vec3f(-0.3, 0.4, -0.3)),
    ]
    render(model, sol, 0.0; lights=qube_lights)[1]
end
##
if false # Don't typically plot when testing
    using Plots
    f1 = plot(sol, idxs=[ssys.qubependulum.elbow_joint.phi, 0.1*ssys.control_system.runtime.swingup_catch.u, ssys.control_system.runtime.swingup_catch.neartop.y])
    hline!([pi], l=(:dash, :black), primary=false)
    f2 = plot(sol, idxs=ssys.qubependulum.shoulder_joint.phi)
    plot(f1, f2) |> display

    ##


    plot(sol, idxs=[
        # ssys.control_system.u,
        ssys.control_system.runtime.swingup_catch.energyswingup.realoutput,
        ssys.control_system.runtime.swingup_catch.energyswingup.limiter.u,
        # ssys.qubependulum.torquesource.tau,
    ])


    plot(sol, idxs=[
        ssys.control_system.runtime.swingup_catch.velocityestimator_elbow.vel
        ssys.control_system.runtime.swingup_catch.velocityestimator_elbow.discretederivative.y
        ssys.qubependulum.elbow_joint.w

        ssys.control_system.runtime.swingup_catch.velocityestimator_shoulder.vel
        ssys.control_system.runtime.swingup_catch.velocityestimator_shoulder.discretederivative.y
        ssys.qubependulum.shoulder_joint.w
    ])
end
sol[ssys.qubependulum.upper_arm.m]
sol[ssys.qubependulum.lower_arm.m]
# sol[ssys.qubependulum.lower_arm.I_11]



##
# Design the upright-stabilizing LQR feedback gain. `design_lqr` linearizes the plant
# about the upright equilibrium (control loop opened at the u_plant/shoulder_y/elbow_y
# analysis points), discretizes, and solves the LQR problem. Encapsulated in
# QuanserComponents so this script and the `FurutaSwingupExperiment` codegen analysis share one
# implementation. `Q1` is the state-penalty diagonal in the order
# [shoulder_angle, elbow_angle, shoulder_velocity, elbow_velocity]; `Q2` is the control
# penalty.
Ts = 0.005
@time L = QuanserComponents.design_lqr(; Ts, Q1 = [1000.0, 10.0, 1.0, 1.0], Q2 = 100.0)
@show L


# bodeplot(ss(ControlSystemsBase.linearize(furuta, [0,pi,0,0], [0.0], pendulum_parameters(), 0)..., I, 0), plotphase=false)
# bodeplot!(P, plotphase=false)


##
# Code generation for the swing-up controller.
#
# `QuanserComponents.generate_swingup_controller` / `SwingupController` compile the
# discrete `SwingupWithHoming` controller into a standalone SynchJulia node (Julia- or
# C-executable), and `export_swingup_c` writes C sources. The test below drives the
# generated controller in closed loop against the Dyad `QubePendulum`, discretized with
# `SeeToDee.Rk4`. On the real rig the same node instead runs `open_hardware!(:hil)`, with
# no Julia in the loop.

using SeeToDee
using StaticArrays
import DyadCompilerPasses

@testset "codegen" begin
    Ts = 0.005

    # The controller does its own I/O by calling into csrc/qube_hw.c, so tests point
    # that library at a Julia handler instead of the board. `hold(sh, el)` feeds fixed
    # angles and records what the controller writes.
    function hold(shoulder, elbow)
        applied = Ref(NaN)
        QuanserComponents.bind_hardware!(measure = () -> (shoulder, elbow),
                                        control = u -> (applied[] = u))
        applied
    end

    # ---- generated controller: compile once, run on both backends ----------
    @testset "compile and step ($backend)" for backend in (:julia, :c)
        ctrl = QuanserComponents.SwingupController(; Ts, backend)
        # No log open: `log_row` is a no-op returning 0, so the program runs unchanged.
        @test ctrl.log.columns == QuanserComponents.SWINGUP_LOG_COLUMNS
        # near hanging -> small command; near upright -> stabilizer engages
        applied = hold(0.0, 0.01)
        out = ctrl()
        @test out.row == 0                               # no log open
        @test out.shoulder == 0.0 && out.elbow == 0.01   # what the node measured
        @test isfinite(out.u) && out.u == applied[]      # and what it wrote
        SynchToolkit.reset!(ctrl)
        applied = hold(0.0, Float64(π))
        out_top = ctrl()
        @test isfinite(out_top.u) && out_top.u == applied[]
        # one read and one write per tick, no more
        @test QuanserComponents.hardware_counters() == (n_measure = 1, n_write = 1)
        # a tick that does not fire touches no hardware
        out_notick = ctrl(; tick = false)
        @test QuanserComponents.hardware_counters() == (n_measure = 1, n_write = 1)
        @test out_notick.u === nothing
    end

    # :julia and :c backends must produce identical control signals. This is the
    # single most important property of the ccall-based hardware I/O: the same
    # controller definition, the same C implementation of the I/O, both targets.
    let cj = QuanserComponents.SwingupController(; Ts, backend=:julia),
        cc = QuanserComponents.SwingupController(; Ts, backend=:c)
        step_all(c) = [(hold(0.0, el); c().u) for el in (0.01, 0.5, 1.5, Float64(π))]
        @test step_all(cj) ≈ step_all(cc)
    end

    # ---- C source export ----------------------------------------------------
    @testset "C export" begin
        dir = mktempdir()
        r = QuanserComponents.export_swingup_c(dir; Ts)
        @test isfile(joinpath(dir, "top.c"))
        @test isfile(joinpath(dir, "top.h"))
        csrc = read(joinpath(dir, "top.c"), String)
        @test occursin("$(r.mangled)_step", csrc)
        @test occursin("$(r.mangled)_reset", csrc)

        # The node does its own I/O, so the exported C must declare and call the
        # hardware entry points by name -- not bake in a Julia function pointer.
        for sym in ("qube_hw_measure", "qube_hw_shoulder", "qube_hw_elbow", "qube_hw_write")
            @test occursin("extern double $sym(", csrc)
        end
        @test !occursin("(int64_t)0x", csrc)     # no baked-in pointer

        # ... and the implementation of those symbols ships alongside it, logging included:
        # the program writes its own rows, through the same csrc/qube_log.c the Julia
        # backend calls.
        @test occursin("extern double qube_log_row(", csrc)
        for f in ("qube_hw.c", "qube_hw.h", "qube_log.c", "qube_log.h")
            @test isfile(joinpath(dir, f))
        end

        # the runnable control loop is emitted alongside the node sources. It is only
        # timing and logging now: no hardware calls of its own.
        @test isfile(joinpath(dir, "run_hardware.c"))
        @test isfile(joinpath(dir, "Makefile"))
        hsrc = read(joinpath(dir, "run_hardware.c"), String)
        # The loop is a checked-in C file copied verbatim, so it refers to the controller
        # only through macros; the mangled symbols live in the generated config header.
        @test occursin("QUBE_STEP(", hsrc)
        @test occursin("qube_hw_open(QUBE_HW_MODE_HIL, QUBE_ARM0)", hsrc)
        @test !occursin("hil_read_encoder", hsrc)
        @test !occursin(r.mangled, hsrc)          # nothing controller-specific baked in
        @test isfile(joinpath(dir, "run_hardware_config.h"))
        cfg = read(joinpath(dir, "run_hardware_config.h"), String)
        @test occursin("#define QUBE_STEP  $(r.mangled)_step", cfg)
        @test occursin("#define QUBE_RESET $(r.mangled)_reset", cfg)
        @test occursin("#define QUBE_MEM   $(r.mangled)_mem", cfg)
        @test occursin("#define QUBE_OUT   $(r.mangled)_out", cfg)
        @test occursin("gains_words[]", cfg) && occursin("auto_words[]", cfg)
        # The loop knows nothing about *which* program it runs: it never reads the node's
        # output struct, and the log it opens is the one the model's `DataLogger` was built
        # with. That is what lets the same checked-in harness serve the friction experiment
        # too (see the friction C-export testset below).
        @test !occursin("QUBE_OUT_", cfg) && !occursin("QUBE_OUT_", hsrc)
        @test !occursin("fopen", hsrc)
        @test occursin("qube_log_open(QUBE_LOG_FILE, QUBE_LOG_HEADER, QUBE_LOG_NCOLS)", hsrc)
        @test occursin("#define QUBE_LOG_FILE   \"run_hardware.csv\"", cfg)
        @test occursin("#define QUBE_LOG_NCOLS  8", cfg)
        @test occursin("time\\tshoulder_angle\\telbow_angle", cfg)   # tab-escaped header
        # where the Quanser HIL SDK and a C compiler are present, the harness must build
        # (static-linked, so no hardware needs to be connected). This is what proves the
        # node's `extern qube_hw_*` declarations resolve against qube_hw.c.
        if isdir("/opt/quanser/hil_sdk") &&
           (Sys.which("cc") !== nothing || Sys.which("gcc") !== nothing)
            exe = QuanserComponents.compile_hardware_harness(dir)
            @test isfile(exe)
        end
    end

    # `FurutaSwingupBase` must stay `partial` in dyad/swingup_experiment.dyad. Without it the
    # compiler emits generated/FurutaSwingupBase_definition.jl, which redefines
    # `FurutaSwingupBaseSpec` and adds a second, self-recursive `run_analysis` for it. The
    # hand-written implementation is included later so it wins at runtime and everything
    # still appears to work — but the overwrite disables precompilation of the whole
    # package. This has been silently reintroduced twice by an editor round-trip, hence the
    # guard.
    @testset "the analysis bases stay partial" begin
        DI = QuanserComponents.DyadInterface
        @test length(methods(DI.run_analysis,
                             (QuanserComponents.FurutaSwingupBaseSpec,))) == 1
        @test length(methods(DI.run_analysis,
                             (QuanserComponents.FurutaFrictionBaseSpec,))) == 1
        # `QubeHardwareRunBase` is the root both analyses extend, so un-partialling it is the
        # same hazard one level up: the compiler would emit a `QubeHardwareRunBaseSpec`
        # struct, colliding with the dispatching function of that name in analysis_base.jl.
        @test length(methods(DI.run_analysis,
                             (QuanserComponents.FurutaIdentificationBaseSpec,))) == 1
        for base in ("FurutaSwingupBase", "FurutaFrictionBase", "FurutaIdentificationBase",
                     "QubeHardwareRunBase")
            @test !isfile(joinpath(pkgdir(QuanserComponents), "generated",
                                   "$(base)_definition.jl"))
        end
    end

    # One generated entry point, two analyses: the Dyad compiler resolves an analysis' spec
    # type to the *root* of its `extends` chain, so both analyses' generated code constructs
    # `QubeHardwareRunBaseSpec`. It has to land on the right base spec, and say so rather than
    # guess when it cannot tell.
    @testset "shared analysis base" begin
        QC = QuanserComponents
        @test QC.QubeHardwareRunBaseSpec(; Ts, Q1 = [1.0, 2, 3, 4], Q2 = 3.0) isa
              QC.FurutaSwingupBaseSpec
        @test QC.QubeHardwareRunBaseSpec(; Ts, settle = 0.5) isa QC.FurutaFrictionBaseSpec
        @test QC.QubeHardwareRunBaseSpec(; Ts, traj_file = "x.csv") isa
              QC.FurutaIdentificationBaseSpec
        @test_throws ArgumentError QC.QubeHardwareRunBaseSpec(; Ts)            # ambiguous
        @test_throws ArgumentError QC.QubeHardwareRunBaseSpec(; nonsense = 1)  # unknown
        # Each analysis' shared-parameter defaults are its Dyad partial's, not the root's.
        @test QC.FurutaSwingupBaseSpec().export_c && !QC.FurutaFrictionBaseSpec().export_c
        # Neither analysis pins the card options: the friction sweep has to run on the same
        # command-to-torque path the controller does, or the friction it identifies is not the
        # friction the controller faces (see csrc/qube_hw.h).
        @test QC.FurutaFrictionBaseSpec().card_options == ""
        @test QC.FurutaSwingupBaseSpec().card_options == ""
        @test QC.FurutaSwingupBaseSpec().Tf == 10.0
        # ... and the shared field block really is shared, not copied per analysis.
        shared = (:Ts, :run, :Tf, :umax, :arm_deg, :card_options, :backend, :export_c,
                  :output_dir, :log_file, :deploy_host, :deploy_dir, :live_plot,
                  :live_plot_cmd, :live_plot_config, :model, :overrides)
        for f in shared, T in (QC.FurutaSwingupBaseSpec, QC.FurutaFrictionBaseSpec,
                               QC.FurutaIdentificationBaseSpec)
            @test hasfield(T, f)
        end
    end

    # ---- FurutaSwingupExperiment analysis (Dyad analysis wrapper) --------------------
    # Designs the LQR gain from the penalty weights Q1/Q2, then exports the C.
    @testset "FurutaSwingupExperiment analysis" begin
        DI = QuanserComponents.DyadInterface
        dir = mktempdir()
        # run=false: the analysis defaults to run=true (compile + drive the hardware),
        # which must not happen in the test suite.
        @time sol = QuanserComponents.FurutaSwingupExperiment(; output_dir = dir, Ts, run = false)
        @test isfile(joinpath(dir, "top.c")) && isfile(joinpath(dir, "top.h"))
        @test sol.hwrun.output_dir == dir && !sol.hwrun.ran
        @test !isempty(sol.hwrun.mangled)
        # `run_analysis` may skip the (slow) LQR design, in which case the controller keeps
        # the model's tuned default gain — which is what `sol.L` then reports.
        @test length(sol.L) == 4 && all(isfinite, sol.L)
        # metadata advertises the file-listing artifact, which lists the generated files
        md = DI.AnalysisSolutionMetadata(sol)
        @test any(a -> a.name === :GeneratedFiles, md.artifacts)
        tbl = DI.artifacts(sol, :GeneratedFiles)
        @test Set(tbl.file) == Set(readdir(dir))
        @test occursin("$(sol.hwrun.mangled)_step", join(tbl.symbol))
        @test_throws ArgumentError DI.artifacts(sol, :RunLog)      # nothing ran
        @test_throws ArgumentError DI.artifacts(sol, :Nonexistent)
        # `umax` parameterizes the model rather than being read off the spec: an override on the
        # model's own parameter is what becomes the runtime-settable tunable. Stated through
        # the override map directly, which is what the compiler builds from a
        # `model = FurutaHardware(final umax = umax)` line — the line itself lives in a .dyad
        # file the Dyad GUI rewrites, and it has come back with the binding dropped twice.
        let m = QuanserComponents.FurutaHardware(; name = :m),
            nm = ModelingToolkit.toggle_namespacing(m, false),
            SymT = ModelingToolkit.SymbolicT
            ov = Dict{SymT, SymT}(ModelingToolkit.unwrap(nm.umax) =>
                                  ModelingToolkit.unwrap(Num(5.0)))
            gen5 = QuanserComponents.generate_swingup_controller(; Ts, param_overrides = ov)
            @test gen5.tuning_defaults[:umax] == 5.0
        end
        # With `export_c = false` the analysis would run in-process and export nothing.
        sol_ip = QuanserComponents.FurutaSwingupExperiment(; output_dir = mktempdir(), Ts, run = false,
                                                  export_c = false, deploy_host = "")
        @test sol_ip.hwrun.output_dir === nothing
        @test isempty(DI.AnalysisSolutionMetadata(sol_ip).artifacts)
        @test_throws ArgumentError QuanserComponents.FurutaSwingupExperiment(; Ts, run = false,
                                                                    backend = "fortran")
        # Q1/Q2 are the user-facing knob: changing them changes the designed gain. Only
        # meaningful when the analysis designed one — `design_lqr` costs ~2 min, so it is
        # not called here just to have something to compare against.
        if sol.L !== nothing
            L2 = QuanserComponents.design_lqr(; Ts, Q1 = [1000.0, 10.0, 1.0, 1.0], Q2 = 50.0)
            @test !(L2 ≈ sol.L)
        end
    end

    # ---- the swing-up program logs itself too --------------------------------
    # The whole run log is written by the `DataLogger` inside the program, in the
    # `SWINGUP_LOG_COLUMNS` order: what was measured and applied, plus the loop diagnostics
    # `HardwareDiagnostics` reads out of qube_hw.c. So every target logs the same file through
    # the same code and the timing loop around the program writes nothing.
    @testset "swing-up run log" begin
        logfile = joinpath(mktempdir(), "run.csv")
        ctrl = QuanserComponents.SwingupController(; Ts, backend = :julia, log_file = logfile)
        @test ctrl.log.file == logfile
        applied = hold(0.0, 0.01)
        QuanserComponents.open_log!(ctrl.log)
        SynchToolkit.reset!(ctrl)
        out = nothing
        for _ in 1:10
            out = ctrl()
        end
        QuanserComponents.close_log!()
        @test out.row == 10                        # one row per tick, from inside the tick

        D = QuanserComponents.read_log(logfile)
        @test collect(keys(D)) == Symbol.(QuanserComponents.SWINGUP_LOG_COLUMNS)
        @test length(D.time) == 10
        @test D.control_input[end] == applied[]    # what the program actually wrote
        # `time` is wall clock from the first tick, and `dt` has no predecessor on it.
        @test D.time[1] == 0.0 && issorted(D.time) && D.time[end] > 0
        @test D.dt[1] == 0.0 && all(>(0), D.dt[2:end])
        # `exec` is latched by the motor write, so a nonzero value in the *first* row is what
        # proves the diagnostics are scheduled after it — the dependency token in
        # `HardwareDiagnostics.dep` is doing its job. Read too early, this would be 0.
        @test all(>(0), D.exec)
        @test all(<(0.5 * Ts), D.exec)             # and it is a duration, not a timestamp
        # Callback mode has no encoder hardware, hence no counts.
        @test all(iszero, D.count_shoulder) && all(iszero, D.count_elbow)
    end

    # ---- the in-process timing loop ------------------------------------------
    # `run_program!` is what a hardware run does when it is not exported to C: open the device
    # and the log, tick every Ts, close both. It is shared by both programs, so this exercises
    # it with the swing-up one against a first-order axis (which never swings up — the point
    # here is the loop, not the controller).
    @testset "run_program!" begin
        logfile = joinpath(mktempdir(), "loop.csv")
        ctrl = QuanserComponents.SwingupController(; Ts, backend = :julia, log_file = logfile)
        w, phi = Ref(0.0), Ref(0.0)
        QuanserComponents.bind_hardware!(
            measure = () -> (phi[], 0.0),
            control = u -> (w[] += Ts * u; phi[] += Ts * w[]; nothing))
        r = QuanserComponents.run_program!(ctrl; Tf = 0.25, mode = :callback)
        @test r.ticks == 50
        @test r.rows == r.ticks                    # the program wrote one row per tick
        @test r.log_file == logfile
        @test 0.004 < r.timing.median_dt < 0.008   # the loop kept its period
        # The program logs the period it achieved, so a log fetched from another machine
        # carries the same timing the loop would have measured here.
        D = QuanserComponents.read_log(logfile)
        @test length(D.time) == 50
        @test abs(QuanserComponents._median(D.dt[2:end]) - r.timing.median_dt) < 1e-3
        @test QuanserComponents.log_timing(logfile).median_dt ≈
              QuanserComponents._median(D.dt[2:end])
    end

    # ---- friction experiment: logging from inside the program ----------------
    # `FurutaFriction` puts the whole experiment in the synchronous program, including the
    # log: `DataLogger` calls into csrc/qube_log.c the same way the hardware components
    # call into csrc/qube_hw.c. So the properties to check are that the log is written
    # from inside the tick (one row per tick, no driver involvement) and that both
    # backends write the same file.
    @testset "friction experiment ($backend)" for backend in (:julia, :c)
        logfile = joinpath(mktempdir(), "friction.csv")
        # Explicit gains: this testset is about the logging, and the model's `velocity_pi`
        # values are whatever the loop is currently tuned to.
        ctrl = QuanserComponents.FrictionController(; Ts, backend, log_file = logfile,
                                                     K = 0.05, Ti = 0.5)
        @test ctrl.log.file == logfile          # the model carries the filename, not the driver

        # A first-order axis to close the velocity loop around. Coulomb friction included,
        # so the recovered `kc` below has something to find.
        w, phi = Ref(0.0), Ref(0.0)
        QuanserComponents.bind_hardware!(
            measure = () -> (phi[], 0.0),
            control = u -> (w[] += Ts * (0.0065u - 1e-4 * w[] - 3e-4 * tanh(w[] / 0.5)) / 1.2e-4;
                            phi[] += Ts * w[]; nothing))

        # One full sweep of the reference: 6 speeds in each direction.
        N = round(Int, QuanserComponents.friction_sweep_duration() / Ts)
        QuanserComponents.open_log!(ctrl.log)   # the program's own log settings
        SynchToolkit.reset!(ctrl)
        out = nothing
        for _ in 1:N
            out = ctrl()
        end
        QuanserComponents.close_log!()

        st = QuanserComponents.log_state()
        @test st.rows == N          # exactly one row per tick, written inside the tick
        @test !st.error
        @test out.row == N          # and the program's own count agrees

        D = QuanserComponents.read_friction_log(logfile)
        @test collect(keys(D)) == Symbol.(QuanserComponents.FRICTION_LOG_COLUMNS)
        @test length(D.time) == N
        @test D.time[1] ≈ Ts && D.time[end] ≈ N * Ts     # the program's own elapsed time
        # The staircase: `n_levels` speeds in each direction, reaching both extremes.
        @test maximum(D.w_ref) ≈ 30.0 && minimum(D.w_ref) ≈ -30.0
        @test length(unique(round.(D.w_ref, digits = 6))) == 12
        # Quadratically spaced, not evenly: resolution belongs at low speed, where the
        # Coulomb breakaway and the steep part of the friction curve are. So the gaps
        # between successive levels must grow, while the endpoints stay w_min and w_max.
        levels = sort(unique(round.(filter(>(0), D.w_ref), digits = 6)))
        @test length(levels) == 6
        @test levels[1] ≈ 2.0 && levels[end] ≈ 30.0     # spacing does not move the ends
        gaps = diff(levels)
        @test issorted(gaps) && gaps[end] > 3 * gaps[1]
        # (level/(n-1))^2 exactly, so the levels are predictable.
        @test levels ≈ [2.0 + 28.0 * (k / 5)^2 for k in 0:5]
        # The velocity loop tracked, so the log is of an actual experiment.
        @test cor(D.w_ref, D.shoulder_velocity) > 0.9
        # `log_row` takes eight arguments whatever `n` is; only `n` columns get written.
        @test all(l -> length(split(l, '\t')) == QuanserComponents.FRICTION_LOG_NCOLS,
                  readlines(logfile))
    end

    # `DataLogger`'s inputs are an `n`-long array comprehension padded out to `log_row`'s
    # fixed arity of 8. `n` has to stay structural for the comprehension and the padding
    # loops to be resolved at build time.
    @testset "DataLogger arity" begin
        lg = QuanserComponents.DataLogger(; name = :lg, n = 3, filename = "x.csv")
        eqs = string.(ModelingToolkit.equations(lg))
        @test count(e -> occursin("log_row", e), eqs) == 1     # one row per tick
        @test count(e -> occursin(r"v\(t\)\)\[[1-3]\] ~ .*u\(t\)", e), eqs) == 3
        # The five unused columns are padded, and with a *float* zero: an Int literal here
        # makes SynchToolkit materialise `v` with a mixed-type `vcat`, which the C backend
        # rejects as non-isbits. Hence this assertion rather than trust.
        pad = filter(e -> occursin(r"v\(t\)\)\[[4-8]\] ~", e), eqs)
        @test length(pad) == 5
        @test all(e -> occursin("~ 0.0", e), pad)
    end

    # The friction model the plant now uses for all of its shoulder friction. The
    # coefficients are not separately interpretable, so the properties asserted are
    # properties of the curve.
    @testset "Friction model" begin
        S = ModelingToolkit.Symbolics
        p = QuanserComponents.FrictionParams(; kc = 2e-3, kv = 1e-4, k2 = 1e-5,
                                              k3 = 1e-6, w_tanh = 0.5)
        f = QuanserComponents.FrictionAndBackEMF(; name = :f, params = p)
        us, eqs = ModelingToolkit.unknowns(f), ModelingToolkit.equations(f)
        sym(n) = us[findfirst(u -> string(u) == n, us)]
        rhs(n) = eqs[findfirst(e -> string(e.lhs) == n, eqs)].rhs
        # The component is stateless, so inlining `sw` and the parameter defaults leaves
        # the whole friction law as one expression in `w`; compile that and evaluate it.
        expr = S.substitute(S.substitute(rhs("tau_f(t)"),
                                         Dict(sym("sw(t)") => rhs("sw(t)"))),
                            Dict(default_values(f)))
        tau = S.build_function(expr, sym("w(t)"); expression = Val(false))

        # Odd in w: friction opposes the motion whichever way the axis turns.
        @test tau(3.0) ≈ -tau(-3.0)
        # The smoothed sign means no jump at standstill, unlike a hard `sign`.
        @test tau(0.0) == 0.0
        # Past the smoothing width the Coulomb term is essentially fully developed...
        @test tau(2.0) > 0.9 * p.kc
        # ...and the higher-order terms add to it at speed.
        @test tau(30.0) > p.kc + p.kv * 30
        # `friction_nominal` is purely first-order, so it reproduces a plain damper.
        @test QuanserComponents.friction_nominal.kc == 0
        @test QuanserComponents.friction_nominal.k2 == 0 == QuanserComponents.friction_nominal.k3
        # The identified set must be dissipative over the range it was measured on. That is
        # by construction now -- it is `kt/Rm * u(w)` with nothing subtracted, and the
        # measured command rises with speed -- so this is the assertion that would catch a
        # reintroduced back-EMF subtraction.
        let q = QuanserComponents.friction_identified
            fi = QuanserComponents.FrictionAndBackEMF(; name = :fi, params = q)
            us2, eqs2 = ModelingToolkit.unknowns(fi), ModelingToolkit.equations(fi)
            sym2(n) = us2[findfirst(u -> string(u) == n, us2)]
            rhs2(n) = eqs2[findfirst(e -> string(e.lhs) == n, eqs2)].rhs
            e2 = S.substitute(S.substitute(rhs2("tau_f(t)"),
                                          Dict(sym2("sw(t)") => rhs2("sw(t)"))),
                              Dict(default_values(fi)))
            tq = S.build_function(e2, sym2("w(t)"); expression = Val(false))
            ws = range(1.0, 40.0, length = 200)          # the measured range
            @test q.kv > 0
            @test all(>(0), tq.(ws))                     # always opposes the motion
            @test issorted(tq.(ws))                      # and grows with speed
        end
        # `withparams` replaces only what it is given.
        q = QuanserComponents.withparams(p; kc = 5e-3)
        @test q.kc == 5e-3 && q.kv == p.kv && q.w_tanh == p.w_tanh
    end

    # ---- friction identification --------------------------------------------
    # The fit is what the experiment exists for, so it runs inside the analysis. Exercise it
    # on a synthetic log with known coefficients: constant-velocity segments whose command
    # is the friction model evaluated exactly, so the regression must recover it.
    @testset "friction fit" begin
        motor = QuanserComponents.identified
        g = motor.kt / motor.Rm
        a_true = [0.12, 0.004, 2.0e-5, 1.0e-7]     # command-space [sign, w, sign*w^2, w^3]
        Ts_f = 0.005
        # Six speeds per direction, 2 s each, held exactly constant so `acc` is zero and
        # every sample past `settle` survives selection.
        speeds = [s * w for s in (1, -1) for w in range(2, 30, length = 6)]
        nper = round(Int, 2.0 / Ts_f)
        w = repeat(speeds, inner = nper)
        n = length(w)
        u = a_true[1] .* sign.(w) .+ a_true[2] .* w .+
            a_true[3] .* sign.(w) .* w .^ 2 .+ a_true[4] .* w .^ 3
        log = (; time = collect((1:n) .* Ts_f), w_ref = w, shoulder_angle = cumsum(w) .* Ts_f,
                 shoulder_velocity = w, control_input = u, elbow_angle = zeros(n))

        d = QuanserComponents.friction_data(log)
        # Only step transients are rejected. `settle` accounts for most of it (0.6 s of each
        # 2 s segment), and a little more goes because the acceleration smoothing is
        # zero-phase: the spike at a step bleeds backwards into the tail of the segment
        # before it, which `settle` alone does not cover.
        settle_only = n - 12 * round(Int, 0.6 / Ts_f)
        @test 0.9 * settle_only < count(d.keep) < settle_only
        @test all(iszero, d.w_elbow)          # a still pendulum disturbs nothing

        fit = QuanserComponents.fit_friction(d; motor)
        @test fit.a ≈ a_true rtol=1e-6        # exact data, so the regression is exact
        @test fit.resid_rms < 1e-9
        @test fit.nkeep == count(d.keep) && fit.nsamples == n
        # Torque space is just the command-to-torque gain now; nothing is subtracted.
        @test fit.params.kc ≈ g * a_true[1]
        @test fit.params.kv ≈ g * a_true[2]      # no back-EMF subtraction
        @test fit.params.k2 ≈ g * a_true[3]
        @test fit.params.k3 ≈ g * a_true[4]
        # The report is what gets printed, and it must be paste-ready.
        rep = QuanserComponents.friction_report(fit)
        @test occursin("const friction_identified = withparams(friction_nominal;", rep)
        @test occursin("kc", rep) && occursin("k3", rep)

        # Too little surviving data must warn and return nothing, not throw: losing the
        # trace would be worse than losing the fit.
        few = QuanserComponents.friction_data(log; w_min_fit = 1e4)
        @test count(few.keep) == 0
        @test (@test_logs (:warn,) match_mode=:any QuanserComponents.fit_friction(few)) ===
              nothing
    end

    # ---- FurutaFrictionExperiment analysis ----------------------------------
    @testset "FurutaFrictionExperiment analysis" begin
        DI = QuanserComponents.DyadInterface
        RB = QuanserComponents.RecipesBase
        # Same guard as FurutaSwingupBase: `partial` must survive `dyad format`.
        @test length(methods(DI.run_analysis,
                             (QuanserComponents.FurutaFrictionBaseSpec,))) == 1
        @test !isfile(joinpath(pkgdir(QuanserComponents), "generated",
                               "FurutaFrictionBase_definition.jl"))

        dir = mktempdir()
        logfile = joinpath(dir, "analysis.csv")
        # run=false: the analysis defaults to run=true (drive the hardware), which must not
        # happen in the test suite. It still compiles the program.
        sol = QuanserComponents.FurutaFrictionExperiment(; Ts, run = false,
                                                          log_file = logfile,
                                                          output_dir = dir)
        @test sol.log_file == logfile
        @test !sol.hwrun.ran
        @test sol.fit === nothing && sol.data === nothing
        @test isempty(DI.AnalysisSolutionMetadata(sol).artifacts)   # nothing to show yet
        @test_throws ArgumentError DI.artifacts(sol, :Trace)
        @test_throws ArgumentError DI.artifacts(sol, :Nonexistent)
        @test_throws ArgumentError QuanserComponents.FurutaFrictionExperiment(;
            Ts, run = false, backend = "fortran")

        # With a log present, `run = false` re-fits it without touching the hardware — the
        # workflow that used to need a separate script.
        ctrl = QuanserComponents.FrictionController(; Ts, backend = :julia,
                                                     log_file = logfile,
                                                     K = 0.05, Ti = 0.5)
        wr, phi = Ref(0.0), Ref(0.0)
        QuanserComponents.bind_hardware!(
            measure = () -> (phi[], 0.0),
            control = u -> (wr[] += Ts * (0.0065u - 1e-4 * wr[] -
                                          3e-4 * tanh(wr[] / 0.5)) / 1.2e-4;
                            phi[] += Ts * wr[]; nothing))
        QuanserComponents.open_log!(ctrl.log)
        SynchToolkit.reset!(ctrl)
        for _ in 1:round(Int, QuanserComponents.friction_sweep_duration() / Ts)
            ctrl()
        end
        QuanserComponents.close_log!()

        # Gains stated explicitly rather than inherited: the analysis now leaves `K`/`Ti`
        # at 0 meaning "use the model's", and the model's are whatever the loop is
        # currently tuned to, which a test must not depend on.
        sol2 = QuanserComponents.FurutaFrictionExperiment(; Ts, run = false,
                                                           log_file = logfile,
                                                           output_dir = dir,
                                                           K = 0.05, Ti = 0.5)
        @test sol2.data !== nothing
        @test sol2.fit !== nothing
        # The simulated axis has Coulomb friction, so the fit must find some.
        @test sol2.fit.params.kc > 0
        @test sol2.fit.nkeep > 100

        # Artifacts: the two plots, the trace table, the parameter table, and the object.
        md = DI.AnalysisSolutionMetadata(sol2)
        @test Set(nameof.(md.artifacts)) ==
              Set([:ExperimentPlot, :Trace, :FitPlot, :FrictionParameters, :FrictionParams])
        tbl = DI.artifacts(sol2, :Trace)
        @test collect(keys(tbl)) == Symbol.(QuanserComponents.FRICTION_LOG_COLUMNS)
        pars = DI.artifacts(sol2, :FrictionParameters)
        @test length(pars.torque) == 4 && length(pars.command) == 4
        @test DI.artifacts(sol2, :FrictionParams) === sol2.fit.params
        @test DI.artifacts(sol2, :FrictionParams) isa QuanserComponents.FrictionParams

        # `plot(sol)` covers every panel without arguments. Applied through RecipesBase
        # directly so this needs no plotting backend: four panels (three trace + one fit),
        # each series carrying its subplot index.
        series = RB.apply_recipe(Dict{Symbol, Any}(), sol2)
        # 5 experiment series (ω pair + selected scatter, command, diagnostics pair +
        # threshold lines) and 3 fit series (data, hard-sign curve, tanh curve).
        @test length(series) == 8
        @test Set(s.plotattributes[:subplot] for s in series) == Set(1:4)
        # Restricted to what each plot artifact shows.
        exp_series = RB.apply_recipe(Dict{Symbol, Any}(:friction_panels => :experiment), sol2)
        @test maximum(s -> s.plotattributes[:subplot], exp_series) == 3
        fit_series = RB.apply_recipe(Dict{Symbol, Any}(:friction_panels => :fit), sol2)
        @test maximum(s -> s.plotattributes[:subplot], fit_series) == 1
        @test_throws ArgumentError RB.apply_recipe(
            Dict{Symbol, Any}(:friction_panels => :nonsense), sol2)
        # ...and end to end through Plots, which is what the user actually calls and what
        # the two PlotlyPlot artifacts return.
        @test Plots.plot(sol2) isa Plots.Plot
        @test DI.artifacts(sol2, :ExperimentPlot) isa Plots.Plot
        @test DI.artifacts(sol2, :FitPlot) isa Plots.Plot

        # `show` prints the fitted parameters, so the numbers are never more than a
        # `sol` away.
        shown = sprint(show, MIME"text/plain"(), sol2)
        @test occursin("identified friction", shown)
        @test occursin("friction_identified = withparams", shown)
        @test occursin(@sprintf("%.8g", sol2.fit.params.kc), shown)
    end

    # ---- the friction experiment as standalone C ----------------------------
    # What consolidating the harness bought: the friction experiment can be exported, built and
    # deployed exactly like the swing-up controller, because the timing loop no longer knows
    # which program it carries. Nothing about the experiment needed changing for this.
    @testset "friction experiment C export" begin
        dir = mktempdir()
        gen = QuanserComponents.generate_friction_controller(; Ts,
                                    log_file = QuanserComponents.FRICTION_LOG_FILE)
        r = QuanserComponents.export_program_c(gen, dir;
                                    Tf = QuanserComponents.friction_sweep_duration())
        @test isfile(joinpath(dir, "top.c")) && !isempty(r.mangled)
        cfg = read(joinpath(dir, "run_hardware_config.h"), String)
        # The log defines follow *this* program's DataLogger, not the swing-up's.
        @test occursin("#define QUBE_LOG_FILE   \"$(QuanserComponents.FRICTION_LOG_FILE)\"", cfg)
        @test occursin("#define QUBE_LOG_NCOLS  $(QuanserComponents.FRICTION_LOG_NCOLS)", cfg)
        @test occursin("time\\tw_ref\\tshoulder_angle", cfg)
        # ... while the loop itself is the same checked-in file, byte for byte.
        @test read(joinpath(dir, "run_hardware.c"), String) ==
              read(joinpath(pkgdir(QuanserComponents), "csrc", "run_hardware.c"), String)
        # Every file the deploy step ships is present, so this could go to the Pi as it stands.
        for f in QuanserComponents.HARNESS_FILES
            @test isfile(joinpath(dir, f))
        end
        if isdir("/opt/quanser/hil_sdk") &&
           (Sys.which("cc") !== nothing || Sys.which("gcc") !== nothing)
            @test isfile(QuanserComponents.compile_hardware_harness(dir))
        end
    end

    # The four targets, as a decision that can be checked without a rig. The case that
    # matters: a named `deploy_host` means the run happens *there*, which is only possible
    # through exported C, so it implies `export_c`. Ignoring the host instead ran the program
    # in this process against whatever card the workstation has — `hil_open` failing with
    # -108 while the QUBE sat attached to the host that was named.
    @testset "which target a run goes to" begin
        T(; kw...) = QuanserComponents.run_target(; kw...)
        pi = "fredrikb@192.168.1.49"
        @test T(run = true,  export_c = false, deploy_host = "") === :inprocess
        @test T(run = true,  export_c = true,  deploy_host = "") === :local_c
        @test T(run = true,  export_c = true,  deploy_host = pi) === :remote_c
        @test T(run = true,  export_c = false, deploy_host = pi) === :remote_c   # implied
        @test T(run = false, export_c = false, deploy_host = "") === :none
        @test T(run = false, export_c = true,  deploy_host = "") === :export_only
        @test T(run = false, export_c = false, deploy_host = pi) === :export_only
        # ...and the log paths follow the same rule, so a deployed run's fetched copy is
        # where a later `run = false` re-fit looks for it.
        spec = QuanserComponents.FurutaFrictionBaseSpec(; output_dir = "friction_c",
                                                         deploy_host = pi,
                                                         log_file = "friction.csv")
        @test QuanserComponents.program_log_path(spec, "x.csv") == "friction.csv"
        @test QuanserComponents.local_log_path(spec, "x.csv") ==
              joinpath("friction_c", "friction.csv")
    end

    # Where a run's log goes, which differs by target: an exported program runs in its own
    # directory on whatever machine it was deployed to, so it gets the bare name; an
    # in-process run resolves a bare name against `output_dir` so the log lands beside
    # everything else the analysis wrote.
    @testset "log paths per target" begin
        spec(; kw...) = QuanserComponents.FurutaFrictionBaseSpec(; kw...)
        P = QuanserComponents.program_log_path
        F = QuanserComponents.FRICTION_LOG_FILE
        @test P(spec(; output_dir = "out", export_c = false), F) == joinpath("out", F)
        @test P(spec(; output_dir = "out", export_c = true), F) == F
        @test P(spec(; output_dir = "out", export_c = true, log_file = "/tmp/a/b.csv"), F) ==
              "b.csv"
        @test P(spec(; output_dir = "out", log_file = "/tmp/a/b.csv"), F) == "/tmp/a/b.csv"
        @test P(spec(; output_dir = "out", log_file = "mine.csv"), F) ==
              joinpath("out", "mine.csv")
    end

    # ---- the identification replay: an input file feeding the program --------
    # The mirror of the log: `TrajectorySource` reads one designed sample per tick from inside
    # the program (csrc/qube_traj.c), so the replay is the program's own doing on every target
    # rather than something a driver feeds it. What has to hold is that the samples arrive in
    # order, one per tick, and identically on both backends.
    @testset "identification replay ($backend)" for backend in (:julia, :c)
        trajfile = joinpath(pkgdir(QuanserComponents), "input_design.csv")
        logfile = joinpath(mktempdir(), "replay.csv")
        ctrl = QuanserComponents.IdentificationController(; Ts, backend, traj_file = trajfile,
                                                          log_file = logfile, umax = 3.0)
        @test ctrl.traj.file == trajfile     # the model carries it, not the driver
        @test ctrl.log.columns == QuanserComponents.IDENTIFICATION_LOG_COLUMNS
        # A free arm: the command accelerates it and nothing restores it, which is what makes
        # this experiment need a supervisor at all.
        w, phi = Ref(0.0), Ref(0.0)
        QuanserComponents.bind_hardware!(
            measure = () -> (phi[], 0.0),
            control = u -> (w[] += Ts * 20u; phi[] += Ts * w[]; nothing))
        @test QuanserComponents.open_traj!(ctrl.traj) == 12000
        QuanserComponents.open_log!(ctrl.log)
        SynchToolkit.reset!(ctrl)
        out = nothing
        for i in 1:200
            out = ctrl()
            @test out.k == i                 # the tick counter is the trajectory index
        end
        QuanserComponents.close_log!()

        D = QuanserComponents.read_log(logfile)
        useq = Float64.(readdlm(trajfile)[2:end, 2])
        @test D.u_des[1:200] ≈ useq[1:200]   # sample for sample, in order
        @test all(iszero, D.tripped)
        @test all(abs.(D.control_input) .<= 3.0)
    end

    # The supervisor is the reason an open-loop replay is safe to run at all, and the latch is
    # what the old driver-side `break` could not do: keep commanding 0 afterwards.
    @testset "safety supervisor" begin
        trajfile = joinpath(pkgdir(QuanserComponents), "input_design.csv")
        ctrl = QuanserComponents.IdentificationController(; Ts, backend = :julia,
                            traj_file = trajfile, log_file = joinpath(mktempdir(), "t.csv"),
                            umax = 3.0)
        arm = Ref(deg2rad(130.0))            # past abort (120)
        applied = Float64[]
        QuanserComponents.bind_hardware!(measure = () -> (arm[], 0.0),
                                        control = u -> push!(applied, u))
        QuanserComponents.open_traj!(ctrl.traj)
        QuanserComponents.open_log!(ctrl.log)
        SynchToolkit.reset!(ctrl)
        o1 = ctrl()
        @test o1.tripped == 1.0 && o1.u == 0.0
        arm[] = 0.0                          # back in bounds: the latch must hold
        o2 = ctrl()
        @test o2.tripped == 1.0 && o2.u == 0.0
        QuanserComponents.close_log!()
        @test all(iszero, applied)

        # Between warn and abort the command is a pull-back towards centre, not the design.
        ctrl2 = QuanserComponents.IdentificationController(; Ts, backend = :julia,
                            traj_file = trajfile, log_file = joinpath(mktempdir(), "w.csv"),
                            umax = 3.0)
        QuanserComponents.bind_hardware!(measure = () -> (deg2rad(100.0), 0.0),
                                        control = u -> nothing)
        QuanserComponents.open_traj!(ctrl2.traj)
        QuanserComponents.open_log!(ctrl2.log)
        SynchToolkit.reset!(ctrl2)
        o = ctrl2()
        @test o.tripped == 0.0
        @test o.u ≈ clamp(-2.0 * deg2rad(100.0), -3.0, 3.0)
        QuanserComponents.close_log!()
    end

    # A run longer than its trajectory coasts rather than holding the last sample, and says so
    # afterwards -- the alternative would be a program that keeps driving on stale data.
    @testset "past the end of the trajectory" begin
        dir = mktempdir()
        short = joinpath(dir, "short.csv")
        write(short, "time\tu\n0.0\t1.0\n0.005\t2.0\n")
        ctrl = QuanserComponents.IdentificationController(; Ts, backend = :julia,
                            traj_file = short, log_file = joinpath(dir, "l.csv"))
        QuanserComponents.bind_hardware!(measure = () -> (0.0, 0.0), control = u -> nothing)
        @test QuanserComponents.open_traj!(ctrl.traj) == 2
        QuanserComponents.open_log!(ctrl.log)
        SynchToolkit.reset!(ctrl)
        @test ctrl().u_des == 1.0
        @test ctrl().u_des == 2.0
        @test ctrl().u_des == 0.0
        @test QuanserComponents.traj_state().error
        QuanserComponents.close_log!()
    end

    @testset "FurutaIdentificationExperiment analysis" begin
        DI = QuanserComponents.DyadInterface
        dir = mktempdir()
        trajfile = joinpath(pkgdir(QuanserComponents), "input_design.csv")
        sol = QuanserComponents.FurutaIdentificationExperiment(; Ts, run = false,
                            output_dir = dir, traj_file = trajfile, deploy_host = "")
        # The duration comes off the file rather than being stated twice.
        @test sol.nsamples == 12000
        @test !sol.hwrun.ran && !sol.tripped
        @test isfile(joinpath(dir, "top.c")) && isfile(joinpath(dir, "qube_traj.c"))
        # The samples travel with the sources, and the harness is told to open them: that is
        # what lets a replay run on the machine the QUBE is attached to.
        @test isfile(joinpath(dir, basename(trajfile)))
        cfg = read(joinpath(dir, "run_hardware_config.h"), String)
        @test occursin("#define QUBE_TRAJ_FILE   \"$(basename(trajfile))\"", cfg)
        @test occursin("#define QUBE_TRAJ_COLUMN 2", cfg)
        @test occursin("#define QUBE_LOG_NCOLS  8", cfg)
        @test Set(nameof.(DI.AnalysisSolutionMetadata(sol).artifacts)) == Set([:GeneratedFiles])
        @test_throws ArgumentError DI.artifacts(sol, :Trace)
        if isdir("/opt/quanser/hil_sdk") &&
           (Sys.which("cc") !== nothing || Sys.which("gcc") !== nothing)
            @test isfile(QuanserComponents.compile_hardware_harness(dir))
        end
        @test occursin("trajectory:", sprint(show, MIME"text/plain"(), sol))
    end

    # ---- the live plot must not open the previous run's log ------------------
    # Measured, not assumed: a kst launched while the previous run's log is still in place
    # binds to that content, and a truncate-and-refill underneath it never reaches the
    # window -- it keeps *reading* the file the whole time, so read activity proves nothing.
    # `launch_live_plot` therefore starts the viewer through a wrapper that waits for a log
    # that is non-empty *and* newer than the wrapper. This checks that rule with a fake
    # viewer, so it needs no plotting program.
    @testset "live plot waits for this run's log" begin
        dir = mktempdir()
        started = joinpath(dir, "started")
        viewer = joinpath(dir, "fake_viewer")
        write(viewer, "#!/bin/sh\ndate +%s.%N > $started\nsleep 30\n")
        chmod(viewer, 0o755)
        cfg = joinpath(dir, "session.kst")
        write(cfg, "<kst><datavector field=\"time\"/></kst>")
        csv = joinpath(dir, "run.csv")
        # The previous run's log, sitting where this run's will go.
        write(csv, "time\tvalue\n1.0\t2.0\n")
        sleep(0.05)

        p = QuanserComponents.launch_live_plot(dir; cmd = viewer, config = cfg,
                                               log = "run.csv", wait_for_log = 20.0)
        @test p !== nothing
        try
            sleep(1.0)
            @test !isfile(started)          # stale log: the viewer must still be waiting
            # This run writes its log; the wrapper may now let the viewer go.
            write(csv, "time\tvalue\n")
            t_write = time()
            # Non-empty, not merely present: the shell creates the file before writing to it.
            ok = timedwait(() -> isfile(started) && !isempty(strip(read(started, String))), 10.0)
            @test ok === :ok
            @test parse(Float64, strip(read(started, String))) >= t_write - 0.5
        finally
            kill(p)
        end
    end

    # ---- closed loop: Dyad plant discretized with SeeToDee.Rk4 --------------
    # The plant must use the same parameter set as the one the controller is tuned for,
    # i.e. the one `FurutaSwingup` instantiates. `QubePendulum()` would default to
    # `nominal`, and the controller does not stabilize that plant at all: the identified
    # set differs by 5.3x in Jp and 3.6x in mr, which is well outside the tuning's margin.
    @testset "swingup — Dyad plant (SeeToDee)" begin
        @named world = MultibodyComponents.World(render=false)
        @named plant = QuanserComponents.QubePendulum(idparams = QuanserComponents.identified)
        @named plantmodel = System(Equation[], ModelingToolkit.t_nounits; systems=[world, plant])
        # Compile like `multibody()` (LDIV solves the mass matrix -> explicit ODE),
        # declaring the motor voltage as the control input.
        ssys_plant = mtkcompile(plantmodel; inputs=[plant.voltage],
            MultibodyComponents.linsys..., optimize=[DyadCompilerPasses.LDIV_RULE])
        res = ModelingToolkit.generate_control_function(ssys_plant, [plant.voltage])
        f_oop = res.f[1]; io_sys = res.io_sys; dvs = res.dvs
        meas = ModelingToolkit.build_explicit_observed_function(io_sys,
            [plant.shoulder_angle, plant.elbow_angle]; inputs=[plant.voltage])
        pp = ModelingToolkit.MTKParameters(io_sys, Dict(plant.voltage => 0.0))
        f_disc = SeeToDee.Rk4(f_oop, Ts; supersample=4)

        ctrl = QuanserComponents.SwingupController(; Ts, backend=:julia)
        N = round(Int, 12.0 / Ts)
        x0 = zeros(length(dvs))
        x0[2] = deg2rad(0.15)   # near hanging
        elbow = Float64[]
        k = Ref(0)
        # The controller reads and writes through csrc/qube_hw.c; point that at the
        # discretized plant. `measure` observes the current state, `control` advances it.
        # The state lives in a `Ref` so the handlers mutate rather than rebind: assigning a
        # captured variable would need a `global` declaration to work when this block is
        # pasted into the REPL, and such a declaration is lexical (not conditional), which
        # then breaks it inside the testset function.
        xr = Ref(x0)
        QuanserComponents.bind_hardware!(
            measure = function ()
                y = meas(xr[], [0.0], pp, k[]*Ts)
                (y[1], y[2])
            end,
            control = function (u)
                xr[] = f_disc(xr[], [u], pp, k[]*Ts)
                k[] += 1
            end)
        for i in 1:N
            out = ctrl()
            @test isfinite(out.u) && abs(out.u) <= 10
            push!(elbow, mod(out.elbow, 2π))
        end
        @test QuanserComponents.hardware_counters() == (n_measure = N, n_write = N)
        near_top = abs.(elbow .- π) .< 0.4
        @test any(near_top)                       # reaches upright
        @test all(near_top[end-40:end])           # and stays there (stabilized)
    end


end
