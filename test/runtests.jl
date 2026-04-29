using QuanserComponents
using Test
using ModelingToolkit
using MultibodyComponents
using DiscreteComponents
using SynchToolkit
    
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
hline!([pi], l=(:dash, :black), primary=false)|> display
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
sol[ssys.qubependulum.lower_arm.I]



##


Rm = 8.4
kt = 0.042
km = 0.042

mr = 0.095
r  = 0.085
Jr = mr*r^2/3
br = 0.05e-3

mp = 0.024
Lp = 0.129
l  = Lp/2

Jp = mp*Lp^2/3
bp = 0.05*5e-5
g = 9.81