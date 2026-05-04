using QuanserComponents
using Test
using ModelingToolkit
using MultibodyComponents
using DiscreteComponents
using SynchToolkit
using ControlSystemsMTK
using ControlSystemsBase
using LinearAlgebra
    
# include("../generated/tests.jl")


@named model = QuanserComponents.FurutaSwingup()
ssys = multibody(model, additional_passes=[SynchToolkit.compile_lustre])
prob = ODEProblem(ssys, [
    ssys.qubependulum.elbow_joint.phi => 0.98pi
    ssys.gain.k => 1.0
], (0.0, 2.0))



using OrdinaryDiffEqTsit5

sol = solve(prob, Tsit5())
all(isnan, sol[ssys.swingup.elbow_angle])

using Plots
# plot(sol)
# hline!([pi], l=(:dash, :black), primary=false)|> display


plot(sol, idxs=[ssys.qubependulum.elbow_joint.phi, ssys.swingup.u, ssys.swingup.neartop.y])
hline!([pi], l=(:dash, :black), primary=false) |> display
##
import GLMakie
render(model, sol, 0.0)[1]
##


plot(sol, idxs=ssys.swingup.u)


plot(sol, idxs=[
    ssys.swingup.velocityestimator_elbow.vel
    ssys.swingup.velocityestimator_elbow.discretederivative.y
    ssys.qubependulum.elbow_joint.w
])

sol[ssys.qubependulum.upper_arm.m]
sol[ssys.qubependulum.lower_arm.m]
# sol[ssys.qubependulum.lower_arm.I]



##
# Linearize the plant about the upright equilibrium with the controller loop
# opened at the u_plant analysis point, then design an LQR feedback gain.


op = Dict(
    ssys.qubependulum.elbow_joint.phi    => π,
    ssys.qubependulum.shoulder_joint.phi => 0.0,
    ssys.qubependulum.elbow_joint.w      => 0.0,
    ssys.qubependulum.shoulder_joint.w   => 0.0,
)

P = named_ss(model, [ssys.u_plant], [ssys.shoulder_y, ssys.elbow_y];
    op,
    loop_openings = [ssys.u_plant, ssys.shoulder_y, ssys.elbow_y],
    allow_input_derivatives = false,
    warn_empty_op = true,
)
@show state_names(P)

Ts = 0.01
Pd = c2d(ss(P), Ts)

# Q1 weights are in the order printed by `state_names(P)`. The QuanserInterface
# example uses [θ_shoulder, φ_elbow, θ̇, φ̇] -> [1000, 10, 1, 1]; permute the
# diagonal to match `state_names(P)` if the order differs.
Q1 = Diagonal([1000.0, 10.0, 1.0, 1.0])
Q2 = 10.0 * I(1)

L = lqr(Pd, Q1, Q2)
@show L
