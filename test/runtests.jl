using QuanserComponents
using Test
using ModelingToolkit
using MultibodyComponents
# using OrdinaryDiffEq
using DiscreteComponents
using SynchToolkit
    
include("../generated/tests.jl")


@named model = QuanserComponents.FurutaPendulum()
ssys = multibody(model, additional_passes=[SynchToolkit.compile_lustre])
prob = ODEProblem(ssys, [], (0.0, 10.0))