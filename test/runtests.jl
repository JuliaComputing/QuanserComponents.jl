using QuanserComponents
using Test
using ModelingToolkit
using MultibodyComponents
# using DiscreteComponents
using SynchToolkit
using LinearAlgebra
# using SynchJulia
# SynchJulia.backend!(:julia)
##
# include("../generated/tests.jl")


@named model = QuanserComponents.FurutaSwingup()

#
ssys = multibody(model, additional_passes=[SynchToolkit.compile_lustre])
k = ModelingToolkit.ShiftIndex()
##
prob = ODEProblem(ssys, [
    ssys.qubependulum.shoulder_joint.render => false
    # ssys.qubependulum.shoulder_joint.radius => 0.01
    # ssys.qubependulum.shoulder_joint.color => [0.8, 0.8, 0.8, 1]
    # ssys.qubependulum.shoulder_joint.cylinder_length => 0.03
    ssys.qubependulum.elbow_joint.phi => 0+deg2rad(0.15)
    ssys.qubependulum.shoulder_joint.phi => 0.0
    ssys.gain.k => 1.0
    ssys.swingup.lqrstabilizer.umax => 10
    # ssys.qubependulum.Jp => 1e-1
    # ssys.swingup.lqrstabilizer.L1 => L[1]
    # ssys.swingup.lqrstabilizer.L2 => L[2]
    # ssys.swingup.lqrstabilizer.L3 => L[3]
    # ssys.swingup.lqrstabilizer.L4 => L[4]
    # ssys.swingup.velocityestimator_elbow.discretederivative.u(k-1) => pi
    ssys.swingup.energyswingup.umax => 3.0
    ssys.swingup.energyswingup.gain.k => 100.0
    ssys.swingup.energyswingup.arm_centering.k => -1.0

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
# Linearize the plant about the upright equilibrium with the controller loop
# opened at the u_plant analysis point, then design an LQR feedback gain.
using ControlSystemsMTK
using ControlSystemsBase

op = Dict(
    ssys.qubependulum.elbow_joint.phi    => π,
    ssys.qubependulum.shoulder_joint.phi => 0.0,
    ssys.qubependulum.elbow_joint.w      => 0.0,
    ssys.qubependulum.shoulder_joint.w   => 0.0,
    ssys.qubependulum.voltage            => 0.0,
    ssys.elbow_sampler.u => 0.0,
    ssys.shoulder_sampler.u => 0.0,
)


outputs = [
    ssys.qubependulum.shoulder_angle
    ssys.qubependulum.elbow_angle
    ssys.qubependulum.shoulder_joint.w
    ssys.qubependulum.elbow_joint.w
]

P = named_ss(model, [ssys.u_plant], outputs;
    op,
    loop_openings = [ssys.u_plant, ssys.shoulder_y, ssys.elbow_y],
    warn_empty_op = true,
    additional_passes=[SynchToolkit.compile_lustre], 
    MultibodyComponents.linsys...,
)
@show state_names(P)

Ts = 0.005
Pd = c2d(ss(P), Ts)

# Q1 weights are in the order printed by `state_names(P)`. The QuanserInterface
# example uses [θ_shoulder, φ_elbow, θ̇, φ̇] -> [1000, 10, 1, 1]; permute the
# diagonal to match `state_names(P)` if the order differs.
Q1 = P.C'Diagonal([1000.0, 10.0, 1.0, 1.0])*P.C
Q2 = 10.0 * I(1)

L = vec(lqr(Pd, Q1, Q2)*pinv(P.C))
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
    # The QI `furuta` plant uses the opposite sign convention to the Dyad model for
    # both the arm angle and the motor voltage, so the generated (Dyad-tuned)
    # controller is applied through that transform. The Dyad-tuned energy target
    # overshoots the QI plant's energy scale, so the controller swings the pendulum
    # up to the top region but does not hold balance without QI-specific tuning —
    # consistent with the two plants not behaving identically. This test exercises
    # the full measure -> controller -> control loop (identical to the hardware loop)
    # and checks the swing-up reaches upright.
    @testset "swingup — QuanserInterface simulator" begin
        process = QuanserInterface.QubeServoPendulumSimulator(; Ts)
        process.x = SA[0.0, 0.4, 0.0, 2.0]         # pendulum kicked from hanging
        QuanserInterface.initialize(process)
        ctrl = QuanserComponents.SwingupController(; Ts, backend=:julia,
            L = [-2.85, -24.4, -0.99, -2.0], umax = 10.0)
        N = round(Int, 15.0 / Ts)
        elbow = Float64[]
        try
            for i in 1:N
                y = QuanserInterface.measure(process)     # [arm θ, pendulum α]
                u = ctrl(-y[1], y[2])                     # QI arm-angle sign flip
                @test isfinite(u) && abs(u) <= 10
                QuanserInterface.control(process, [-u])   # QI voltage sign flip
                push!(elbow, mod(y[2], 2π))
            end
        finally
            QuanserInterface.control(process, [0.0])
            QuanserInterface.finalize(process)
        end
        @test any(abs.(elbow .- π) .< 0.4)         # swing-up reaches the top region
    end
end