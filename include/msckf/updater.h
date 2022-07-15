#ifndef MSCKF_MSCKF_UPDATER_H
#define MSCKF_MSCKF_UPDATER_H

#include "types/type_all.h"
#include "utils/utils.h"
#include "msckf/state.h"
#include "feature/Feature.h"
#include <fstream>

namespace msckf_dvio
{

class Updater {

public:
  Updater(priorDvl prior_dvl, paramMsckf param_msckf);

  void updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D);

  void updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D, bool is_simple);

  void updatePressure(std::shared_ptr<State> state, const double pres_begin, const double pres_curr, bool is_simple);
  
  void marginalizeDvl(std::shared_ptr<State> state);

  void marginalize(std::shared_ptr<State> state, SubStateName clone_name);

  void updateCam(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &features);

private:
  /// prior information for DVL
  priorDvl prior_dvl_;

  paramMsckf param_msckf_;

  //! TEST:
  long long int count;
  const char *file_path="/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_updater.csv";
};

} // end of namespace msckf_dvio 

#endif // MSCKF_MSCKF_UPDATER_H
