module QuanserComponentsSynchJuliaCExt

import QuanserComponents
import SynchJuliaC

QuanserComponents.c_backend() = SynchJuliaC.CBackend()
QuanserComponents.synchjuliac_export_c(dir, node) = SynchJuliaC.export_c(dir, node)
QuanserComponents.synchjuliac_mangle(name, types::Type...) = SynchJuliaC.mangle(name, types...)
QuanserComponents.synchjuliac_cc() = SynchJuliaC.cc()

end # module
