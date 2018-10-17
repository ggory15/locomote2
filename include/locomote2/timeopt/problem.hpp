#ifndef __locomote2_timeopt_problem_hpp__
#define __locomote2_timeopt_problem_hpp__

#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <stdexcept>

#include "locomote2/timeopt/fwd.hpp"
#include "locomote2/timeopt/phase.hpp"

# include <hpp/timeopt/momentumopt/dynopt/DynamicsOptimizer.hpp>
# include <hpp/timeopt/momentumopt/cntopt/ContactPlanFromFootPrint.hpp>


namespace locomote2
{
  namespace timeopt
  {
    struct ProblemInfo
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef std::vector< PhaseInfo,Eigen::aligned_allocator<PhaseInfo> > PhaseInfoVector;
      typedef std::vector<std::string> StringVector;
      typedef size_t Index;
      typedef Eigen::Vector3d Vector3d;
      typedef Eigen::VectorXd VectorXd;
      typedef Eigen::Matrix3d Matrix3d;
      typedef Eigen::Quaterniond Quaternion;

      ProblemInfo();
      ProblemInfo(const Index size);
      
      Index numPhases() const { return phases.size(); }
      Vector3d initCOM() const {return init_com;}
      Vector3d finalCOM() const {return final_com;}

      bool generateDATFile(const std::string & filename) const throw (std::invalid_argument);
      
      // for using hpp-timeoptimization 
      void setTimeoptSolver(const std::string cfg_path, const std::string file_name);
      void setInitialState() const throw (std::invalid_argument);
      void setFinalState() const throw (std::invalid_argument);
      void setContactSequence();
      void solve();

      double getTimetrajectory(const int & cnt){
        return time_traj(cnt); 
      }
      Vector3d getCOMtrajectory(const int & cnt){
        return dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).centerOfMass();
      }
      Vector3d getLMOMtrajectory(const int & cnt){
        return dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).linearMomentum();
      }
      Vector3d getAMOMtrajectory(const int & cnt){
        return dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).angularMomentum();
      }
      Vector3d getContactForce(const int & id, const int & cnt){
        return dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).endeffectorForce(id);
      }
      int getNumSize() {return dyn_optimizer_.numtimestep(); }

      PhaseInfoVector phases;     
      Vector3d init_com, final_com;
      double mass;
      VectorXd time_traj;

      private: 
        hpp::timeopt::PlannerSetting planner_setting_;
        hpp::timeopt::DynamicsState dynamic_state_;
        hpp::timeopt::DynamicsSequence ref_sequence_;
        hpp::timeopt::ContactPlanFromFootPrint contact_plan_;
        hpp::timeopt::DynamicsOptimizer dyn_optimizer_;
        
    }; // struct ProblemInfo
  }
}

#endif // ifndef __locomote_muscod_problem_hpp__
