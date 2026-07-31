module QuanserComponents
using DiscreteComponents
using MultibodyComponents
using LinearAlgebra

# Spec bases for the analyses. Both `FurutaSwingupExperiment` and `FurutaFrictionExperiment` extend the
# partial analysis `QubeHardwareRunBase`, and the Dyad compiler resolves an analysis' spec type
# to the root of that chain, so the generated definitions
# (generated/Furuta{Swingup,Friction}Experiment_definition.jl) refer to
# `AbstractQubeHardwareRunBaseSpec` and `QubeHardwareRunBaseSpec`; both must exist before the
# generated module is included.
include("analysis_base.jl")

# The hardware-I/O and data-logging operators must exist before the generated module is
# loaded: the components in dyad/ are written in terms of them.
include("hardware_io.jl")
include("data_log.jl")

include("../generated/module.jl")

# Building a synchronous program for the rig and running it, in three layers: what every
# program shares (program.jl), how one gets onto hardware (harness.jl), and the two programs
# themselves (codegen.jl for the swing-up controller, friction.jl for the friction experiment).
include("program.jl")
include("harness.jl")
include("codegen.jl")
include("friction.jl")

# The analyses on top of them.
include("swingup_analysis.jl")
include("friction_analysis.jl")

end # module QuanserComponents
