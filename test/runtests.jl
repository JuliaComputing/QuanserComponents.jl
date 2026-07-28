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
    # now come from the model defaults, tuned for the QuanserInterface-matched plant.

    ssys.qubependulum.floor.r_shape => [0, -0.10, 0]

    # ssys.qubependulum.base_box.color => [0.1, 0.1, 0.1, 1]
    # ssys.qubependulum.base_box.shape.specular_coefficient => 1.5

    # ssys.qubependulum.base_box.shape_transform => MultibodyComponents.Rp2T(MultibodyComponents.RotXYZ(-pi/2, 0, -pi / 2), [0, -0.075, 0])
    # ssys.qubependulum.motor_front_mesh.shape_transform => MultibodyComponents.Rp2T(MultibodyComponents.RotY(-pi / 2) * MultibodyComponents.RotX(-pi / 2), [0.025, 0, 0])


    # ssys.qubependulum.lower_arm.shape_transform => MultibodyComponents.Rp2T(MultibodyComponents.RotY(pi / 2) * MultibodyComponents.RotX(pi / 2), [0, -0.058, 0])

], (0.0, 10.0))



using OrdinaryDiffEqLowOrderRK

@time "solve" sol = solve(prob, BS3(), dt=0.005)
@assert !all(isnan, sol[ssys.swingup.elbow_angle])

import GLMakie
using GLMakie: Makie, AmbientLight, SpotLight, PointLight, RGBf, Vec3f, Vec2f


@test abs(mod2pi(sol(sol.t[end], idxs=ssys.qubependulum.elbow_joint.phi))) - pi < 0.01

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

##
using Plots
f1 = plot(sol, idxs=[ssys.qubependulum.elbow_joint.phi, 0.1*ssys.swingup.runtime.swingup.u, ssys.swingup.runtime.swingup.neartop.y])
hline!([pi], l=(:dash, :black), primary=false)
f2 = plot(sol, idxs=ssys.qubependulum.shoulder_joint.phi)
plot(f1, f2) |> display

##


plot(sol, idxs=[
    # ssys.swingup.u,
    ssys.swingup.runtime.swingup.energyswingup.realoutput,
    ssys.swingup.runtime.swingup.energyswingup.limiter.u,
    # ssys.qubependulum.torquesource.tau,
])


plot(sol, idxs=[
    ssys.swingup.runtime.swingup.velocityestimator_elbow.vel
    ssys.swingup.runtime.swingup.velocityestimator_elbow.discretederivative.y
    ssys.qubependulum.elbow_joint.w

    ssys.swingup.runtime.swingup.velocityestimator_shoulder.vel
    ssys.swingup.runtime.swingup.velocityestimator_shoulder.discretederivative.y
    ssys.qubependulum.shoulder_joint.w
])

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
# Code generation for the Swingup controller.
#
# `QuanserComponents.generate_swingup_controller` / `SwingupController` compile the
# discrete `Swingup` controller into a standalone SynchJulia node (Julia- or
# C-executable), and `export_swingup_c` writes C sources. The tests below drive the
# generated controller in closed loop against two plants:
#   A) the Dyad `QubePendulum`, discretized with `SeeToDee.Rk4`, and
#   B) `QuanserInterface.QubeServoPendulumSimulator` (the same interface the real
#      `QubeServoPendulum` hardware uses).
# The two plants use different parameters/sign conventions, so they do not behave
# identically (see notes in test B).

using SeeToDee
using StaticArrays
using QuanserInterface
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
        @test occursin("$(r.mangled)_step", hsrc)
        @test occursin("qube_hw_open(QUBE_HW_MODE_HIL, ARM0)", hsrc)
        @test !occursin("hil_read_encoder", hsrc)
        # where the Quanser HIL SDK and a C compiler are present, the harness must build
        # (static-linked, so no hardware needs to be connected). This is what proves the
        # node's `extern qube_hw_*` declarations resolve against qube_hw.c.
        if isdir("/opt/quanser/hil_sdk") &&
           (Sys.which("cc") !== nothing || Sys.which("gcc") !== nothing)
            exe = QuanserComponents.compile_hardware_harness(dir)
            @test isfile(exe)
        end
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
        @test length(sol.L) == 4 && all(isfinite, sol.L)
        # metadata advertises the file-listing artifact, which lists the generated files
        md = DI.AnalysisSolutionMetadata(sol)
        @test any(a -> a.name === :GeneratedFiles, md.artifacts)
        tbl = DI.artifacts(sol, :GeneratedFiles)
        @test Set(tbl.file) == Set(readdir(dir))
        @test occursin("$(sol.mangled)_step", join(tbl.symbol))
        @test_throws ArgumentError DI.artifacts(sol, :Nonexistent)
        # Q1/Q2 are the user-facing knob: changing them changes the designed gain
        L2 = QuanserComponents.design_lqr(; Ts, Q1 = [1000.0, 10.0, 1.0, 1.0], Q2 = 50.0)
        @test !(L2 ≈ sol.L)
    end

    # ---- Test A: Dyad plant discretized with SeeToDee.Rk4 -------------------
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

    # ---- Test B: QuanserInterface simulator (hardware-compatible interface) --
    # An independent implementation of the same plant: QuanserInterface's `furuta`
    # equations, in its own parameterization and sign conventions. Driving the SAME
    # generated controller — with no per-plant sign flips or retuning — through it is what
    # catches convention errors. Bound through the shim's callback backend; on the real rig
    # the same node instead runs `open_hardware!(:hil)` with no Julia in the loop.
    #
    # The simulator's own defaults are the datasheet (nominal) values, so it has to be
    # given the identified set the controller is tuned for. Two parameters need converting:
    # QI's `Jr` is the arm inertia about the pivot, while `QubePendulum` carries the arm's
    # central inertia plus a CoM radius; QI's `Jp` is about the pendulum CoM, matching
    # `QubePendulum`'s. Read the values off the component so this cannot drift from the
    # model. (Sanity check: feeding `nominal` through this mapping reproduces the
    # simulator's default behaviour exactly.)
    @testset "swingup — QuanserInterface simulator" begin
        @named refplant = QuanserComponents.QubePendulum(idparams = QuanserComponents.identified)
        let ic = ModelingToolkit.initial_conditions(refplant),
            np = ModelingToolkit.toggle_namespacing(refplant, false)
            val(s) = Float64(Symbolics.value(ic[ModelingToolkit.unwrap(s)]))
            global qi_p = (Rm = val(np.Rm), kt = val(np.kt), km = val(np.km),
                           mr = val(np.mr), r = val(np.r),
                           Jr = val(np.Jr) + val(np.mr) * val(np.r_cm_r)^2,  # about the pivot
                           br = val(np.br), mp = val(np.mp), Lp = val(np.Lp),
                           l = val(np.l), Jp = val(np.Jp), bp = val(np.bp), g = 9.81)
        end
        process = QuanserInterface.QubeServoPendulumSimulator(; Ts, p = qi_p)
        process.x = SA[0.0, deg2rad(0.15), 0.0, 0.0]   # near hanging
        QuanserInterface.initialize(process)
        ctrl = QuanserComponents.SwingupController(; Ts, backend=:julia)
        QuanserComponents.bind_hardware!(
            measure = () -> QuanserInterface.measure(process),   # [arm θ, pendulum α]
            control = u -> QuanserInterface.control(process, [u]))
        N = round(Int, 15.0 / Ts)
        elbow = Float64[]
        try
            for i in 1:N
                out = ctrl()
                @test isfinite(out.u) && abs(out.u) <= 10
                push!(elbow, mod(out.elbow, 2π))
            end
        finally
            QuanserComponents.close_hardware!()
            QuanserInterface.finalize(process)
        end
        @test QuanserComponents.hardware_counters() == (n_measure = N, n_write = N)
        near_top = abs.(elbow .- π) .< 0.4
        @test any(near_top)                        # reaches upright
        @test all(near_top[end-100:end])           # and stabilizes there
    end

end
