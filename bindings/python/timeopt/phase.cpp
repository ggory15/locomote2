
#include "locomote2/bindings/python/timeopt/expose-timeopt.hpp"
#include "locomote2/bindings/python/timeopt/phase.hpp"

namespace locomote2
{
  namespace python
  {
    void exposeTimeoptPhase()
    {
      TimeoptPhasePythonVisitor<locomote2::timeopt::PhaseInfo>::expose("TimeoptPhase");
    }
  }
}
