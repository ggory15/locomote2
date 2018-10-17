#ifndef __locomote2_python_geometry_ellipsoid_hpp__
#define __locomote2_python_geometry_ellipsoid_hpp__

namespace locomote2 {
  namespace python {
    namespace internal {
      
      template<typename T>
      struct build_type_name
      {
        static const char* name();
        static const char* shortname();
      };
      
      template<>
      const char * build_type_name<double>::shortname() { return "d"; }
      
 
    }
  }
}

#endif // ifndef __test_timeopt_python_geometry_ellipsoid_hpp__
