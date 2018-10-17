
#include "locomote2/bindings/python/timeopt/expose-timeopt.hpp"
#include "locomote2/bindings/python/timeopt/problem.hpp"

namespace locomote2
{
  namespace python
  {
    void exposeTimeoptProblem()
    {
      TimeoptProblemPythonVisitor<locomote2::timeopt::ProblemInfo>::expose("TimeoptProblem");
    }
  }
}
