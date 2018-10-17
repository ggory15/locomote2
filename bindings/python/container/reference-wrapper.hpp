#ifndef __locomote2_python_container_reference_wrapper_hpp__
#define __locomote2_python_container_reference_wrapper_hpp__

#include <boost/python.hpp>
#include <boost/ref.hpp>
#include <boost/python/to_python_indirect.hpp>
#include <typeinfo>

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename reference_wrapper_type>
    struct reference_wrapper_converter
    {
      typedef reference_wrapper_type ref_type;
      typedef typename reference_wrapper_type::type type;
      
      static PyObject* convert(ref_type const& ref)
      {
        ref_type* const p = &const_cast<ref_type&>(ref);
        if(p == 0)
          return bp::detail::none();
        return bp::detail::make_reference_holder::execute(&p->get());
      }
      
      static void expose()
      {
        bp::to_python_converter<ref_type,reference_wrapper_converter>();
      }
    };
  }
}

#endif // ifndef __locomote_python_container_reference_wrapper_hpp__
