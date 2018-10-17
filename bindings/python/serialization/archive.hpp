#ifndef __locomote2_python_serialization_archive_hpp__
#define __locomote2_python_serialization_archive_hpp__

#include <string>
#include <boost/python.hpp>

namespace locomote2
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Derived>
    struct SerializableVisitor
    : public boost::python::def_visitor< SerializableVisitor<Derived> >
    {
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def("saveAsText",&Derived::saveAsText,bp::args("filename"),"Saves *this inside a text file.")
        .def("loadFromText",&Derived::loadFromText,bp::args("filename"),"Loads *this from a text file.")
        .def("saveAsXML",&Derived::saveAsXML,bp::args("filename","tag_name"),"Saves *this inside a XML file.")
        .def("loadFromXML",&Derived::loadFromXML,bp::args("filename","tag_name"),"Loads *this from a XML file.")
        .def("saveAsBinary",&Derived::saveAsBinary,bp::args("filename"),"Saves *this inside a binary file.")
        .def("loadFromBinary",&Derived::loadFromBinary,bp::args("filename"),"Loads *this from a binary file.")
        ;
      }

    };
  }
}

#endif // ifndef __locomote_python_serialization_archive_hpp__
