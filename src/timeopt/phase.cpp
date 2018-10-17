#include "locomote2/timeopt/phase.hpp"

namespace locomote2
{
  namespace timeopt
  {
    PhaseInfo::PhaseInfo()
    : ee_id(EE_Undefined)
    , start_time(-1.)
    , end_time(-1.)
    , pos(Vector3d(0,0,0))
    , rot(Matrix3d::Zero())
    {}
  
    PhaseInfo::PhaseInfo(const PhaseInfo & other)
    : ee_id(other.ee_id)
    , start_time(other.start_time)
    , end_time(other.end_time)
    , pos(other.pos)
    , rot(other.rot)
    {}
  }
}
