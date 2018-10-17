#ifndef __timeopt_python_timeopt_phase_hpp__
#define __timeopt_python_timeopt_phase_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>

#include "locomote2/timeopt/phase.hpp"
#include "locomote2/bindings/python/container/visitor.hpp"

namespace locomote2
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Phase>
    struct TimeoptPhasePythonVisitor
    : public boost::python::def_visitor< TimeoptPhasePythonVisitor<Phase> >
    {
      typedef typename Phase::Index Index;
      typedef typename Phase::VectorX VectorX;
      typedef typename Phase::MatrixX MatrixX;
      typedef typename Phase::VectorXVector VectorXVector;
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<Phase>(bp::args("other"),"Copy constructor."))
        //.def(bp::init<Index,Index,Index>(bp::args("num_nodes","state_dim","control_dim"),"Default constructor."))      
        .def_readwrite("ee_id",&Phase::ee_id,"Endeffector ID")
        .def_readwrite("start_time",&Phase::start_time,"start_time of the phase.")
        .def_readwrite("end_time",&Phase::end_time,"end_time of the phase.")
        .def_readwrite("pos",&Phase::pos,"Foot Position")
        .def_readwrite("rot",&Phase::rot,"Foot Rotation.")
                
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("assign",&assign,bp::args("other"),"Copy other into *this.")        
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Timeopt phase info.";
        bp::class_<Phase>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TimeoptPhasePythonVisitor<Phase>())
        ;
        
        // Expose related quantities
        VectorPythonVisitor<VectorXVector,true>::expose("StdVec_VectorXd");
        eigenpy::enableEigenPySpecific<VectorX,VectorX>();
        eigenpy::enableEigenPySpecific<MatrixX,MatrixX>();
      }
      
    protected:
      
      static void assign(Phase & self, const Phase & other) { self = other; }
      
    };
  }
}


#endif // ifndef __timeopt_python_timeopt_phase_hpp__
