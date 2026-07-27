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
f1 = plot(sol, idxs=[ssys.qubependulum.elbow_joint.phi, 0.1*ssys.swingup.u, ssys.swingup.neartop.y])
hline!([pi], l=(:dash, :black), primary=false)
f2 = plot(sol, idxs=ssys.qubependulum.shoulder_joint.phi)
plot(f1, f2) |> display

##


plot(sol, idxs=[
    # ssys.swingup.u,
    ssys.swingup.energyswingup.realoutput,
    ssys.swingup.energyswingup.limiter.u,
    # ssys.qubependulum.torquesource.tau,
])


plot(sol, idxs=[
    ssys.swingup.velocityestimator_elbow.vel
    ssys.swingup.velocityestimator_elbow.discretederivative.y
    ssys.qubependulum.elbow_joint.w

    ssys.swingup.velocityestimator_shoulder.vel
    ssys.swingup.velocityestimator_shoulder.discretederivative.y
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

    # ---- generated controller: compile once, run on both backends ----------
    @testset "compile and step ($backend)" for backend in (:julia, :c)
        ctrl = QuanserComponents.SwingupController(; Ts, backend)
        # near hanging -> small command; near upright -> stabilizer engages
        @test isfinite(ctrl(0.0, 0.01))
        SynchToolkit.reset!(ctrl)
        u_top = ctrl(0.0, Float64(π))
        @test isfinite(u_top)
    end

    # :julia and :c backends must produce identical control signals
    let cj = QuanserComponents.SwingupController(; Ts, backend=:julia),
        cc = QuanserComponents.SwingupController(; Ts, backend=:c)
        uj = [cj(0.0, el) for el in (0.01, 0.5, 1.5, Float64(π))]
        uc = [cc(0.0, el) for el in (0.01, 0.5, 1.5, Float64(π))]
        @test uj ≈ uc
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

        # the runnable hardware control loop is emitted alongside the node sources
        @test isfile(joinpath(dir, "run_hardware.c"))
        @test isfile(joinpath(dir, "Makefile"))
        hsrc = read(joinpath(dir, "run_hardware.c"), String)
        @test occursin("$(r.mangled)_step", hsrc)
        @test occursin("hil_read_encoder", hsrc)
        # where the Quanser HIL SDK and a C compiler are present, the harness must build
        # (static-linked, so no hardware needs to be connected)
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
    @testset "swingup — Dyad plant (SeeToDee)" begin
        @named world = MultibodyComponents.World(render=false)
        @named plant = QuanserComponents.QubePendulum()
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
        x = zeros(length(dvs)); x[2] = deg2rad(0.15)   # near hanging
        elbow = Float64[]; u = 0.0
        for i in 1:N
            y = meas(x, [u], pp, (i-1)*Ts)
            u = ctrl(y[1], y[2])
            @test isfinite(u) && abs(u) <= 10
            x = f_disc(x, [u], pp, (i-1)*Ts)
            push!(elbow, mod(y[2], 2π))
        end
        near_top = abs.(elbow .- π) .< 0.4
        @test any(near_top)                       # reaches upright
        @test all(near_top[end-40:end])           # and stays there (stabilized)
    end

    # ---- Test B: QuanserInterface simulator (hardware-compatible interface) --
    # `QubePendulum` is matched to the QuanserInterface hardware-calibrated model
    # (sign conventions and pendulum inertia), so the SAME generated controller —
    # with no per-plant sign flips or retuning — swings up and stabilizes the QI
    # simulator too. The loop is identical to the one that runs on the real
    # `QubeServoPendulum` hardware (measure -> controller -> control).
    @testset "swingup — QuanserInterface simulator" begin
        process = QuanserInterface.QubeServoPendulumSimulator(; Ts)
        process.x = SA[0.0, deg2rad(0.15), 0.0, 0.0]   # near hanging
        QuanserInterface.initialize(process)
        ctrl = QuanserComponents.SwingupController(; Ts, backend=:julia)
        N = round(Int, 15.0 / Ts)
        elbow = Float64[]
        try
            for i in 1:N
                y = QuanserInterface.measure(process)     # [arm θ, pendulum α]
                u = ctrl(y[1], y[2])
                @test isfinite(u) && abs(u) <= 10
                QuanserInterface.control(process, [u])
                push!(elbow, mod(y[2], 2π))
            end
        finally
            QuanserInterface.control(process, [0.0])
            QuanserInterface.finalize(process)
        end
        near_top = abs.(elbow .- π) .< 0.4
        @test any(near_top)                        # reaches upright
        @test all(near_top[end-100:end])           # and stabilizes there
    end


    # ---- Test C: hardware I/O inside the synchronous program ----------------
    # `FurutaHardware` wraps the same `SwingupWithHoming` state machine, but the
    # encoder read and the amplifier write are done by `HardwareMeasurement` /
    # `HardwareCommand` from inside the compiled node instead of by the caller.
    # Driving it against the same simulator must therefore reproduce test B
    # exactly -- same plant, same controller, same order of operations -- and
    # must perform exactly one read and one write per tick.
    @testset "swingup — hardware IO inside the program" begin
        Tf = 15.0
        N = round(Int, Tf / Ts)
        seed() = SA[0.0, deg2rad(0.15), 0.0, 0.0]   # near hanging, as in test B

        # Reference: the ordinary controller with the I/O in the loop.
        ref = let process = QuanserInterface.QubeServoPendulumSimulator(; Ts)
            process.x = seed()
            c = QuanserComponents.SwingupController(; Ts, backend = :julia)
            D = Matrix{Float64}(undef, 3, N)
            for i in 1:N
                y = QuanserInterface.measure(process)
                u = c(y[1], y[2])
                QuanserInterface.control(process, [u])
                D[:, i] = [y[1], y[2], u]
            end
            D
        end

        process = QuanserInterface.QubeServoPendulumSimulator(; Ts)
        process.x = seed()
        io = QuanserComponents.HardwareIO()
        ctrl = QuanserComponents._make_hardware_runtime(
            QuanserComponents.generate_hardware_controller(; Ts); io)
        QuanserComponents.bind_hardware!(ctrl;
            measure = () -> QuanserInterface.measure(process),
            control = u -> QuanserInterface.control(process, [u]))
        SynchToolkit.reset!(ctrl)
        @test io.n_measure == 0 && io.n_control == 0   # reset! clears the counters

        D = Matrix{Float64}(undef, 3, N)
        try
            for i in 1:N
                out = ctrl()
                @test isfinite(out.u) && abs(out.u) <= 10
                D[:, i] = [out.shoulder, out.elbow, out.u]
            end
        finally
            QuanserInterface.control(process, [0.0])
            QuanserInterface.finalize(process)
        end

        # Exactly one hardware access of each kind per tick.
        @test io.n_measure == N
        @test io.n_control == N

        # And the same closed-loop trajectory as the external loop (only
        # floating-point reassociation may differ).
        @test D ≈ ref rtol = 0 atol = 1e-9

        # A tick that does not fire touches no hardware.
        m0, c0 = io.n_measure, io.n_control
        out = ctrl(; tick = false)
        @test io.n_measure == m0 && io.n_control == c0
        @test out.shoulder === nothing && out.elbow === nothing && out.u === nothing
    end
end
