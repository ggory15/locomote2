#ifndef __locomote2_python_timeopt_problem_hpp__
#define __locomote2_python_timeopt_problem_hpp__

#include <boost/python.hpp>
#include <string>

#include "locomote2/timeopt/problem.hpp"
#include "locomote2/bindings/python/container/visitor.hpp"

namespace locomote2
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename Problem>
    struct TimeoptProblemPythonVisitor
    : public boost::python::def_visitor< TimeoptProblemPythonVisitor<Problem> >
    {
      typedef typename Problem::Index Index;
      typedef typename Problem::PhaseInfoVector PhaseInfoVector;
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor"))
        .def(bp::init<Index>(bp::args("size"),"Default constructor with given size."))
        .def_readwrite("phases",&Problem::phases)
        .def_readwrite("init_com",&Problem::init_com)
        .def_readwrite("final_com",&Problem::final_com)
        .def_readwrite("mass", &Problem::mass)       
        .def("numPhases",&Problem::numPhases,"Returns the number of phases contained in *this")
        .def("generateDATFile",&Problem::generateDATFile,bp::args("filename"),"Writes the content of *this in a Timeopt DAT file.")
        .def("setTimeoptSolver", &Problem::setTimeoptSolver, bp::args("filename"), bp::args("filename"), "Load Configuration File for TimeoptSolver")   
        .def("setInitialState", &Problem::setInitialState, "Write the initial state of robot")
        .def("solve", &Problem::solve, "Solve Centroidal Dynamic by hpp-timeoptimization")

        .def("get_trajsize", &Problem::getNumSize, "get Trajectory length")
        .def("get_time", &Problem::getTimetrajectory, bp::args("cnt"), "Get resultant time trajectory" )
        .def("get_COM", &Problem::getCOMtrajectory, bp::args("cnt"), "Get resultant com trajectory" )
        .def("get_LMOM", &Problem::getLMOMtrajectory, bp::args("cnt"), "Get resultant linear momentum trajectory" )
        .def("get_AMOM", &Problem::getAMOMtrajectory, bp::args("cnt"), "Get resultant angular momentum trajectory" )
        .def("get_ContactForce", &Problem::getContactForce, bp::args("id"), bp::args("cnt", "get Contact Force during CD"));
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Timeopt phase info.";
        bp::class_<Problem>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TimeoptProblemPythonVisitor<Problem>())
        ;
        
        // Expose related quantities
        VectorPythonVisitor<PhaseInfoVector>::expose("PhaseInfoVector");
        VectorPythonVisitor<std::string>::expose("StdVec_String");
        
      }
      
    protected:
      
    };
  }
}


#endif // ifndef __locomote_python_muscod_problem_hpp__
