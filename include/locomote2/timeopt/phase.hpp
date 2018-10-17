#ifndef __locomote2_timeopt_phase_hpp__
#define __locomote2_timeopt_phase_hpp__

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include "locomote2/timeopt/fwd.hpp"

namespace locomote2
{
  namespace timeopt
  {
    struct PhaseInfo
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef size_t Index;
      typedef Eigen::DenseIndex DenseIndex;
      typedef double Scalar;
      typedef Eigen::MatrixXd MatrixX;
      typedef Eigen::VectorXd VectorX;
      typedef Eigen::Vector3d Vector3d;
      typedef Eigen::Matrix3d Matrix3d;
      typedef std::vector< VectorX,Eigen::aligned_allocator<VectorX> > VectorXVector;
      
      /// \brief Default constructor
      PhaseInfo();

      /// \brief Copy constructor
      PhaseInfo(const PhaseInfo & other);
      
      /// \brief Copy operator
      PhaseInfo & operator=(const PhaseInfo & other)
      {
        ee_id = other.ee_id;
        start_time = other.start_time;
        end_time = other.end_time;
        pos = other.pos;
        rot = other.rot; 
        
        return *this;
      }
      
      
      /// \brief Comparison operator
      bool operator==(const PhaseInfo & other) const
      {
        return ee_id == other.ee_id
        && start_time == other.start_time
        && end_time == other.end_time
        && pos == other.pos
        && rot == other.rot
        ;
      }
      
      bool operator!=(const PhaseInfo & other) const
      { return !(*this == other); }
      
      /* Bounds on the duration */
      double start_time, end_time;

      /* Position & rot*/ 
      Vector3d pos;
      Matrix3d rot; 
                  
      /* End_effector id */
      EndeffectorID ee_id;
      
      /* Integrator */
      PhaseType phase_type;
      
      
    }; // struct PhaseInfo
  }
}

#endif // ifndef __locomote_timeopt_phase_hpp__
