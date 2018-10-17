#ifndef __locomote2_timeopt_fwd_hpp__
#define __locomote2_timeopt_fwd_hpp__

namespace locomote2
{
  namespace timeopt
  {
    enum EndeffectorID{
      RF = 0,
      LF = 1,
      RH = 2,
      LH = 3,
      EE_Undefined
    };
    enum PhaseType{
      SS = 0,
      DS = 1,
      TS = 2,
      PT_Undefined
    };
  }
}

#endif // ifndef __locomote_timeopt_fwd_hpp__
