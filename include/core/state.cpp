#include "state.h"

namespace msckf_dvio {

State::State(double timeoffset, Eigen::VectorXd extrinsic, double scale) {

  int current_coord = 0;

  /******************** State ********************/

  /*==========  IMU ==========*/
  imu_ = std::make_shared<Imu>();

  //// rotation:3, translation:3, velocity:3, gyro bias: 3, accelration bias: 3 
  imu_->setCoordinate(current_coord);
  current_coord += 15;

  /*========== DVL ==========*/
  dvl_extrinsic_ = std::make_shared<PoseJPL>();

  dvl_timeoffset_ = timeoffset;
  dvl_extrinsic_->setValue(extrinsic);
  dvl_scale_ = scale;

  //// extrinsic rotation:3, extrinsic translation:3, timeoffset: 1, scale: 1
  dvl_extrinsic_->setCoordinate(current_coord);
  current_coord += 8;

  /*========== Camera ==========*/
  // cam_extrinsic_ = std::make_shared<PoseJPL>();

  // //// extrinsic rotation:3, extrinsic translation:3, timeoffset: 1
  // cam_extrinsic_->setCoordinate(current_coord);
  // current_coord += 7;

  /******************** Covariance ********************/
  cov_ = 1e-3 * Eigen::MatrixXd::Identity(current_coord, current_coord);


  /*========== DVL ==========*/
  //// priors for dvl imu extrinsic-rotation calibration
  cov_.block(dvl_extrinsic_->coordinate(), dvl_extrinsic_->coordinate(), 3, 3) = 
    std::pow(0.001, 2) * Eigen::MatrixXd::Identity(3, 3);
  //// priors for dvl imu extrinsic-translation calibration
  cov_.block(dvl_extrinsic_->coordinate() + 3, dvl_extrinsic_->coordinate() + 3, 3, 3) =
    std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
  //// priors for dvl imu timeoffset calibration
  cov_(dvl_extrinsic_->coordinate() + 6, dvl_extrinsic_->coordinate() + 6) = std::pow(0.01, 2);
  //// priors for dvl scale calibration
  cov_(dvl_extrinsic_->coordinate() + 7, dvl_extrinsic_->coordinate() + 7) = std::pow(0.01, 2);

  /*========== Camera ==========*/
  // //// priors for cam imu extrinsic-rotation calibration
  // cov_.block(cam_extrinsic_->coordinate(), cam_extrinsic_->coordinate(), 3, 3) = 
  //   std::pow(0.001, 2) * Eigen::MatrixXd::Identity(3, 3);
  // //// priors for cam imu extrinsic-translation calibration
  // cov_.block(cam_extrinsic_->coordinate() + 3, cam_extrinsic_->coordinate() + 3, 3, 3) =
  //   std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
  // //// priors for cam imu timeoffset calibration
  // cov_(cam_extrinsic_->coordinate() + 6, cam_extrinsic_->coordinate() + 6) = std::pow(0.01, 2);

}

}