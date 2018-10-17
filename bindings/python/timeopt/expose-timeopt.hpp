#ifndef __timeopt_python_muscod_expose_muscod_hpp__
#define __timeopt_python_muscod_expose_muscod_hpp__


namespace locomote2
{
  namespace python
  {
    
    void exposeTimeoptPhase();
    void exposeTimeoptEnums();
    void exposeTimeoptProblem();
    
    inline void exposeTimeopt()
    {
      exposeTimeoptEnums();
      exposeTimeoptPhase();
      exposeTimeoptProblem();
    }
    
  }
}

#endif // ifndef __timeopt_python_muscod_expose_muscod_hpp__
