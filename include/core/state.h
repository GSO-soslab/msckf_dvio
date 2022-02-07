#ifndef MSCKF_CORE_STATE_H_
#define MSCKF_CORE_STATE_H_

#include <memory>
#include <map>
// #include <vector>

#include "types/type.h"

namespace msckf_dvio
{

class State {
  
public:
  State(double timeoffset, Eigen::VectorXd extrinsic, double scale);

  ~State() {}


/******************** Variables ********************/

  // Current timestamp (should be the last update time!)
  double timestamp_;

  /*===== IMU =====**/
  std::shared_ptr<Imu> imu_;

  /*===== DVL =====*/
  // clone state of IMU
  std::map<StateID, std::shared_ptr<PoseJPL>> dvl_clones_;
  // Time offset from Temporal calibrtion between DVL and IMU: (t_imu = t_dvl + t_offset)
  double dvl_timeoffset_;
  // Extrinsic calibration between poses DVL and IMU: (R_I_D, p_I_D)
  std::shared_ptr<PoseJPL> dvl_extrinsic_;
  // Scale factor for DVL: effected by sound speed
  double dvl_scale_;

  /*===== Camera =====*/
  // // clone state of IMU
  // std::map<StateID, std::shared_ptr<PoseJPL>> cam_clones_;
  // // Time offset from Temporal calibrtion between Camera and IMU: (t_imu = t_cam + t_offset)
  // double cam_timeoffset_;
  // // Extrinsic calibration between poses Camera and IMU: (R_I_C, p_I_C)
  // std::shared_ptr<PoseJPL> cam_extrinsic_;

  /*===== Covariance =====*/
  Eigen::MatrixXd cov_;

private:

};
  
} // namespace msckf_dvio


#endif //MSCKF_CORE_STATE_H_