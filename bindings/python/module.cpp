
#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>

#include "locomote2/bindings/python/timeopt/expose-timeopt.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(liblocomote2_pywrap)
{
  eigenpy::enableEigenPy();
  
  using namespace locomote2::python;
  exposeTimeopt();
}
