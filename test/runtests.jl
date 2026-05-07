using QuanserComponents
using Test
using ModelingToolkit
using MultibodyComponents
# using DiscreteComponents
using SynchToolkit
using ControlSystemsMTK
using ControlSystemsBase
using LinearAlgebra
    
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

# bundle = generate_executable(model; MultibodyComponents.linsys...)
# prob = ODEProblem(ssys, [], (0.0, 1.0))
# rt = make_runtime(bundle, prob)
# y = SynchToolkit.SynchJulia.step!(bundle.executable, 0.0, rt)