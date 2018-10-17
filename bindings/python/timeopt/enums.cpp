
#include "locomote2/timeopt/fwd.hpp"

#include "locomote2/bindings/python/timeopt/expose-timeopt.hpp"
#include "locomote2/bindings/python/timeopt/enums.hpp"

#include <boost/python/enum.hpp>

namespace locomote2
{
  namespace python
  {
    namespace bp = boost::python;
    
    using namespace locomote2::timeopt;
    
    void exposeEnumEndeffectorID()
    {
      bp::enum_<EndeffectorID>("EndeffectorID")
      .value("RF",RF)
      .value("LF",LF)
      .value("RH",RH)
      .value("LH",LH)
      .value("EE_Undefined",EE_Undefined)
      ;
    }
    
    void exposeEnumPhaseType()
    {
      bp::enum_<PhaseType>("PhaseType")
      .value("SS",SS)
      .value("DS",DS)
      .value("TS",TS)
      ;
    }
    
    void exposeTimeoptEnums()
    {
      exposeEnumEndeffectorID();
      exposeEnumPhaseType();
    }
  }
}
