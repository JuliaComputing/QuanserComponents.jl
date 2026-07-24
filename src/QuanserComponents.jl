module QuanserComponents
using DiscreteComponents
using MultibodyComponents
using LinearAlgebra

# The FurutaExportC analysis spec base. The concrete `analysis FurutaExportC`
# (dyad/furuta_export_c.dyad) is code-generated into generated/FurutaExportC_definition.jl;
# that generated spec subtypes `AbstractFurutaExportCBaseSpec` and forwards to
# `FurutaExportCBaseSpec`, so both must be defined before the generated module is included.
include("export_analysis_base.jl")

include("../generated/module.jl")

include("codegen.jl")
include("export_analysis.jl")

end # module QuanserComponents
