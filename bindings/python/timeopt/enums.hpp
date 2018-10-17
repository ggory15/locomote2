#ifndef __timeopt_python_muscod_enum_hpp__
#define __timeopt_python_muscod_enum_hpp__

#include <boost/python.hpp>
#include <string>

namespace locomote2
{
  namespace python
  {
    
    void exposeEnumEndeffectorID(const std::string & enum_name);
    void exposeEnumPhaseType(const std::string & enum_name);
    
  }
}


#endif // ifndef __timeopt_python_muscod_enum_hpp__
