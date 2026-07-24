module QuanserComponents
using DiscreteComponents
using MultibodyComponents
using LinearAlgebra
# SynchCompiler ≥ 0.4 provides its C compiler through a package extension: loading
# Clang_unified_jll here activates it, so the `:c` backend works out of the box.
using Clang_unified_jll

# The FurutaExportC analysis spec base. The concrete `analysis FurutaExportC`
# (dyad/furuta_export_c.dyad) is code-generated into generated/FurutaExportC_definition.jl;
# that generated spec subtypes `AbstractFurutaExportCBaseSpec` and forwards to
# `FurutaExportCBaseSpec`, so both must be defined before the generated module is included.
include("export_analysis_base.jl")

include("../generated/module.jl")

include("codegen.jl")
include("export_analysis.jl")

end # module QuanserComponents
