#include "locomote2/timeopt/problem.hpp"

#include <fstream>
#include <iomanip>
using namespace hpp::timeopt;

namespace locomote2
{
  namespace timeopt
  {
    
    template<typename Derived>
    inline void streamVector(std::stringstream & ss, const Eigen::MatrixBase<Derived> & vector)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
      assert(vector.size() > 0);
      
      for(Eigen::DenseIndex k = 0; k < vector.size(); ++k)
        ss << k << ": " << vector[k] << std::endl;
    }
    
    ProblemInfo::ProblemInfo()
    : phases() // pre-allocation
    , init_com(Vector3d::Zero())
    , final_com(Vector3d::Zero())
    , mass(0.0)
    {}
    
    ProblemInfo::ProblemInfo(const Index size)
    : phases(size) // pre-allocation
    , init_com(Vector3d::Zero())
    , final_com(Vector3d::Zero())
    , mass(0.0)
    {}
    
    
    bool ProblemInfo::generateDATFile(const std::string & filename) const throw (std::invalid_argument)
    {
      //typedef PhaseInfoVector::const_iterator PhaseInfoConstIterator;
      using namespace std;
      ofstream output_file;
      
      const Index num_phases = phases.size();
      
      output_file.open (filename.c_str (), ios::trunc | ios::out);
      
      if (! output_file.is_open ())
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }     
      
      stringstream output_content;
      output_content << "Contact Sequence Information" << endl;
      output_content << "Right Foot Sequence" << endl;
      int cnt = 0;

      for (Index i =0; i < num_phases; ++i){
        Quaternion quat(phases[i].rot);
        switch (phases[i].ee_id) {
          case RF:
            output_content << "#" << cnt+1 << ":   " ;
            output_content << phases[i].start_time << ", ";
            output_content << phases[i].end_time << ", ";
            output_content << phases[i].pos.transpose() <<". ";
            output_content << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() ;
            output_content << endl;
            cnt++;
            break;          
          case LF:
            break;
          default: 
            break;
        }  
      }

      output_content <<" " << endl;
      output_content << "Left Foot Sequence" << endl;
      cnt = 0;
      for (Index i =0; i < num_phases; ++i){        
        Quaternion quat(phases[i].rot);
        switch (phases[i].ee_id) {
          case LF:
            output_content << "#" << cnt+1 << ":   " ;
            output_content << phases[i].start_time << ", ";
            output_content << phases[i].end_time << ", ";
            output_content << phases[i].pos.transpose() <<", ";
            output_content << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() ;
            output_content << endl;
            cnt++;
            break;
          case RF:
            break;
          default: 
            break;  
        }        
      }
      output_content <<" " << endl;
      output_content << "Left Hand Sequence" << endl;
      cnt = 0;
      for (Index i =0; i < num_phases; ++i){        
        Quaternion quat(phases[i].rot);
        switch (phases[i].ee_id) {
          case LH:
            output_content << "#" << cnt+1 << ":   " ;
            output_content << phases[i].start_time << ", ";
            output_content << phases[i].end_time << ", ";
            output_content << phases[i].pos.transpose() <<", ";
            output_content << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() ;
            output_content << endl;
            cnt++;
            break;
          case RF:
            break;
          default: 
            break;  
        }        
      }

      output_content <<" " << endl;
      output_content << "Right Hand Sequence" << endl;
      cnt = 0;
      for (Index i =0; i < num_phases; ++i){        
        Quaternion quat(phases[i].rot);
        switch (phases[i].ee_id) {
          case RH:
            output_content << "#" << cnt+1 << ":   " ;
            output_content << phases[i].start_time << ", ";
            output_content << phases[i].end_time << ", ";
            output_content << phases[i].pos.transpose() <<", ";
            output_content << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() ;
            output_content << endl;
            cnt++;
            break;
          case RF:
            break;
          default: 
            break;  
        }        
      }

      output_content << " " << endl;
      output_content << "Dynamic Information" << endl;
      output_content << "Init COM: " << " " << init_com.transpose() << endl;
      output_content << "Final COM: " << " " << final_com.transpose() << endl;      
      output_content << "Mass: " << " " << mass << endl;    
      
      output_file << output_content.str();
      output_file.close();
      
      return true;
    }
    void ProblemInfo::setTimeoptSolver(const std::string cfg_path, const std::string default_path){ 
      planner_setting_.initialize(cfg_path, default_path);
      contact_plan_.initialize(planner_setting_);
      
    }
    void ProblemInfo::setInitialState() const throw (std::invalid_argument){
      if (init_com.norm() < 0.001)
      {
        const std::string exception_message("Initial COM does not seem to be a valid value.");
      } 
      if (mass <= 0.001){
        const std::string exception_message("Robot Mass does not seem to be a valid value.");
      }
    }
    void ProblemInfo::setFinalState() const throw (std::invalid_argument){
      if (final_com.norm() < 0.001)
      {
        const std::string exception_message("Final COM does not seem to be a valid value.");
        throw std::invalid_argument(exception_message);
      }         
    } 
    void ProblemInfo::setContactSequence(){
      FootPrints_t footPrints_RF, footPrints_LF, footPrints_LH, footPrints_RH;
      const Index num_phases = phases.size();
      for (Index i =0; i < num_phases; ++i){
        Quaternion quat(phases[i].rot);
        switch (phases[i].ee_id) {
          case RF:
            footPrints_RF.push_back(FootPrint(phases[i].start_time, phases[i].end_time, phases[i].pos, quat, 1));
            break;          
          case LF:
            footPrints_LF.push_back(FootPrint(phases[i].start_time, phases[i].end_time, phases[i].pos, quat, 1));
            break;  
          case RH:
            footPrints_RH.push_back(FootPrint(phases[i].start_time, phases[i].end_time, phases[i].pos, quat, 1));
            break;    
          case LH:
            footPrints_LH.push_back(FootPrint(phases[i].start_time, phases[i].end_time, phases[i].pos, quat, 1));
            break;  
          default: 
            break;
        }  
      }
      if (footPrints_RF.size() > 0){
        ContactSet con_RF(0, footPrints_RF);
        contact_plan_.addContact(con_RF, dynamic_state_);
      }
      if (footPrints_LF.size() > 0){
        ContactSet con_LF(1, footPrints_LF);
        contact_plan_.addContact(con_LF, dynamic_state_);
      }
      if (footPrints_RH.size() > 0){
        ContactSet con_RH(2, footPrints_RH);
        contact_plan_.addContact(con_RH, dynamic_state_);
      }
      if (footPrints_LH.size() > 0){
        ContactSet con_LH(3, footPrints_LH);
        contact_plan_.addContact(con_LH, dynamic_state_);
      }
      dynamic_state_.setTimehorizon(phases[num_phases-1].end_time-0.1);
    }
    void ProblemInfo::solve() {
      setInitialState();
      setFinalState();

      dynamic_state_.fillInitialBodyState(init_com, mass);
      dynamic_state_.setFinalcom(final_com);
      setContactSequence();

      ref_sequence_.resize(std::floor(dynamic_state_.timehorizon()/planner_setting_.get(PlannerDoubleParam_TimeStep)));
      dyn_optimizer_.initialize(planner_setting_, dynamic_state_, &contact_plan_);
      dyn_optimizer_.optimize(ref_sequence_);

      time_traj.resize(getNumSize());
      int size = getNumSize();
      time_traj(0) = dyn_optimizer_.dynamicsSequence().dynamicsState(0).time();
      for (int i=1; i<size; i++)
        time_traj(i) = dyn_optimizer_.dynamicsSequence().dynamicsState(i).time() + time_traj(i-1);
    }
  }
}
