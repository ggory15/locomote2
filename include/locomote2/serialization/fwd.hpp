
#ifndef __locomote2_serialization_fwd_hpp__
#define __locomote2_serialization_fwd_hpp__

#include <boost/serialization/nvp.hpp>

#define BOOST_SERIALIZATION_MAKE_NVP(member) boost::serialization::make_nvp(##member,member)

#endif // ifndef __locomote_serialization_fwd_hpp__
