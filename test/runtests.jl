using QuanserComponents
using Test
# SynchCompiler ≥ 0.4 provides its C compiler through a package extension; the `:c`
# backend and C export used below require Clang_unified_jll to be loaded.
using Clang_unified_jll
using ModelingToolkit
using MultibodyComponents
# using DiscreteComponents
using SynchToolkit
using LinearAlgebra
using Statistics: cor
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
# QuanserComponents so this script and the `FurutaExportC` codegen analysis share one
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
        # near hanging -> small command; near upright -> stabilizer engages
        applied = hold(0.0, 0.01)
        out = ctrl()
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

        # ... and the implementation of those symbols ships alongside it
        @test isfile(joinpath(dir, "qube_hw.c"))
        @test isfile(joinpath(dir, "qube_hw.h"))

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
        # where the Quanser HIL SDK and a C compiler are present, the harness must build
        # (static-linked, so no hardware needs to be connected). This is what proves the
        # node's `extern qube_hw_*` declarations resolve against qube_hw.c.
        if isdir("/opt/quanser/hil_sdk") &&
           (Sys.which("cc") !== nothing || Sys.which("gcc") !== nothing)
            exe = QuanserComponents.compile_hardware_harness(dir)
            @test isfile(exe)
        end
    end

    # `FurutaExportCBase` must stay `partial` in dyad/furuta_export_c.dyad. Without it the
    # compiler emits generated/FurutaExportCBase_definition.jl, which redefines
    # `FurutaExportCBaseSpec` and adds a second, self-recursive `run_analysis` for it. The
    # hand-written implementation is included later so it wins at runtime and everything
    # still appears to work — but the overwrite disables precompilation of the whole
    # package. This has been silently reintroduced twice by an editor round-trip, hence the
    # guard.
    @testset "FurutaExportCBase is partial" begin
        DI = QuanserComponents.DyadInterface
        @test length(methods(DI.run_analysis,
                             (QuanserComponents.FurutaExportCBaseSpec,))) == 1
        @test !isfile(joinpath(pkgdir(QuanserComponents), "generated",
                               "FurutaExportCBase_definition.jl"))
    end

    # ---- FurutaExportC analysis (Dyad analysis wrapper) --------------------
    # Designs the LQR gain from the penalty weights Q1/Q2, then exports the C.
    @testset "FurutaExportC analysis" begin
        DI = QuanserComponents.DyadInterface
        dir = mktempdir()
        # run=false: the analysis defaults to run=true (compile + drive the hardware),
        # which must not happen in the test suite.
        @time sol = QuanserComponents.FurutaExportC(; output_dir = dir, Ts, run = false)
        @test isfile(joinpath(dir, "top.c")) && isfile(joinpath(dir, "top.h"))
        @test !isempty(sol.mangled)
        # `run_analysis` may skip the (slow) LQR design, in which case the controller
        # keeps the model's tuned default gain and `sol.L` is `nothing`.
        @test sol.L === nothing || (length(sol.L) == 4 && all(isfinite, sol.L))
        # metadata advertises the file-listing artifact, which lists the generated files
        md = DI.AnalysisSolutionMetadata(sol)
        @test any(a -> a.name === :GeneratedFiles, md.artifacts)
        tbl = DI.artifacts(sol, :GeneratedFiles)
        @test Set(tbl.file) == Set(readdir(dir))
        @test occursin("$(sol.mangled)_step", join(tbl.symbol))
        @test_throws ArgumentError DI.artifacts(sol, :Nonexistent)
        # Q1/Q2 are the user-facing knob: changing them changes the designed gain. Only
        # meaningful when the analysis designed one — `design_lqr` costs ~2 min, so it is
        # not called here just to have something to compare against.
        if sol.L !== nothing
            L2 = QuanserComponents.design_lqr(; Ts, Q1 = [1000.0, 10.0, 1.0, 1.0], Q2 = 50.0)
            @test !(L2 ≈ sol.L)
        end
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
        @test ctrl.log_file == logfile          # the model carries the filename, not the driver

        # A first-order axis to close the velocity loop around. Coulomb friction included,
        # so the recovered `kc` below has something to find.
        w, phi = Ref(0.0), Ref(0.0)
        QuanserComponents.bind_hardware!(
            measure = () -> (phi[], 0.0),
            control = u -> (w[] += Ts * (0.0065u - 1e-4 * w[] - 3e-4 * tanh(w[] / 0.5)) / 1.2e-4;
                            phi[] += Ts * w[]; nothing))

        # One full sweep of the reference: 6 speeds in each direction.
        N = round(Int, QuanserComponents.friction_sweep_duration() / Ts)
        QuanserComponents.open_log!(logfile;
                                   header = QuanserComponents.FRICTION_LOG_HEADER,
                                   ncols = QuanserComponents.FRICTION_LOG_NCOLS)
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
                            Dict(ModelingToolkit.default_values(f)))
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
                              Dict(ModelingToolkit.default_values(fi)))
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
        # Same guard as FurutaExportCBase: `partial` must survive `dyad format`.
        @test length(methods(DI.run_analysis,
                             (QuanserComponents.FurutaFrictionBaseSpec,))) == 1
        @test !isfile(joinpath(pkgdir(QuanserComponents), "generated",
                               "FurutaFrictionBase_definition.jl"))

        dir = mktempdir()
        logfile = joinpath(dir, "analysis.csv")
        # run=false: the analysis defaults to run=true (drive the hardware), which must not
        # happen in the test suite. It still compiles the program.
        sol = QuanserComponents.FurutaFrictionExperiment(; Ts, run = false,
                                                          log_file = logfile)
        @test sol.log_file == logfile
        @test !sol.ran
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
        QuanserComponents.open_log!(logfile;
                                   header = QuanserComponents.FRICTION_LOG_HEADER,
                                   ncols = QuanserComponents.FRICTION_LOG_NCOLS)
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
