#ifndef MSCKF_MSCKF_UPDATER_H
#define MSCKF_MSCKF_UPDATER_H

#include "types/type_all.h"
#include "utils/utils.h"
#include "msckf/state.h"

namespace msckf_dvio
{

class Updater {

public:
  Updater(priorDvl prior_dvl, paramMsckf param_msckf);

  void updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D);

  void marginalizeDvl(std::shared_ptr<State> state);

private:
  /// prior information for DVL
  priorDvl prior_dvl_;

  paramMsckf param_msckf_;
};

} // end of namespace msckf_dvio 

#endif // MSCKF_MSCKF_UPDATER_H
