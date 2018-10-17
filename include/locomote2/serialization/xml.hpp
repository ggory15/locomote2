
#ifndef __locomote2_serialization_xml_hpp__
#define __locomote2_serialization_xml_hpp__

#include <iostream>

namespace locomote2
{
  namespace serialization
  {
    template<class C>
    struct serialize
    {
      template<class Archive>
      static operator(Archive & ar, C & c, const unsigned int version);
    };
    
  }
  
}

#endif // ifndef __locomote_serialization_xml_hpp__
