#include "state.h"

namespace msckf_dvio {

State::State(const Params &param) {

  params_msckf_ = param.msckf;
  /********************************************************************************/                       
  /************************************ State *************************************/
  /********************************************************************************/
  int curr_id = 0;


  /*==============================  IMU ==============================*/
  SubState imu;
  // rotation
  auto rotation_I_G = std::make_shared<QuatJPL>();
  rotation_I_G->setId(curr_id);
  curr_id += rotation_I_G->getSize();

  // position
  auto position_G_I = std::make_shared<Vec>(3);
  position_G_I->setId(curr_id);
  curr_id += position_G_I->getSize();

  // velocity
  auto velocity_G_I = std::make_shared<Vec>(3);
  velocity_G_I->setId(curr_id);
  curr_id += velocity_G_I->getSize();

  // bias gyro
  auto bias_gyro = std::make_shared<Vec>(3);
  bias_gyro->setId(curr_id);
  curr_id += bias_gyro->getSize();

  // bias acce
  auto bias_acce = std::make_shared<Vec>(3);
  bias_acce->setId(curr_id);
  curr_id += bias_acce->getSize();

  imu[EST_QUATERNION] = rotation_I_G;
  imu[EST_POSITION] = position_G_I;
  imu[EST_VELOCITY] = velocity_G_I;
  imu[EST_BIAS_G] = bias_gyro;
  imu[EST_BIAS_A] = bias_acce;
  state_[SubStateName::IMU] = imu;

  /*==============================  DVL ==============================*/
  SubState dvl;
  // extrinsic_rotation
  if(param.msckf.do_R_I_D){
    auto extrinsic_q_I_D = std::make_shared<QuatJPL>();
    extrinsic_q_I_D->setId(curr_id);
    extrinsic_q_I_D->setValue(param.prior_dvl.extrinsics.head(4));
    curr_id += extrinsic_q_I_D->getSize();

    dvl[EST_QUATERNION] = extrinsic_q_I_D;
  }

  // extrinsic_position: 
  if(param.msckf.do_p_I_D) {
    auto extrinsic_p_I_D = std::make_shared<Vec>(3);
    extrinsic_p_I_D->setId(curr_id);
    extrinsic_p_I_D->setValue(param.prior_dvl.extrinsics.tail(3));
    curr_id += extrinsic_p_I_D->getSize();

    dvl[EST_POSITION] = extrinsic_p_I_D;
  }

  // timeoffset: t_imu = t_dvl + t_offset
  if(param.msckf.do_time_I_D) {
    auto timeoffset_I_D = std::make_shared<Vec>(1);
    timeoffset_I_D->setId(curr_id);
    timeoffset_I_D->setValue(Eigen::MatrixXd::Constant(1,1,param.prior_dvl.timeoffset));
    curr_id += timeoffset_I_D->getSize();

    dvl[EST_TIMEOFFSET] = timeoffset_I_D;
  }

  // scale: "Scale"
  if(param.msckf.do_scale_D) {
    auto scale_D = std::make_shared<Vec>(1);
    scale_D->setId(curr_id);
    scale_D->setValue(Eigen::MatrixXd::Constant(1,1,param.prior_dvl.scale));
    curr_id += scale_D->getSize();

    dvl[EST_SCALE] = scale_D;
  }


  state_[SubStateName::DVL] = dvl;

  /*==============================  CAM0 ==============================*/
  SubState cam0;

  // extrinsic_rotation
  if(param.msckf.do_R_I_C){
    auto extrinsic_q_I_C = std::make_shared<QuatJPL>();
    extrinsic_q_I_C->setId(curr_id);
    extrinsic_q_I_C->setValue(param.prior_cam.extrinsics.head(4));
    curr_id += extrinsic_q_I_C->getSize();

    cam0[EST_QUATERNION] = extrinsic_q_I_C;
  }

  // extrinsic_position: 
  if(param.msckf.do_p_I_C) {
    auto extrinsic_p_I_C = std::make_shared<Vec>(3);
    extrinsic_p_I_C->setId(curr_id);
    extrinsic_p_I_C->setValue(param.prior_cam.extrinsics.tail(3));
    curr_id += extrinsic_p_I_C->getSize();

    cam0[EST_POSITION] = extrinsic_p_I_C;
  }

  // timeoffset: t_imu = t_cam + t_offset
  if(param.msckf.do_time_I_C) {
    auto timeoffset_I_C = std::make_shared<Vec>(1);
    timeoffset_I_C->setId(curr_id);
    timeoffset_I_C->setValue(Eigen::MatrixXd::Constant(1,1,param.prior_cam.timeoffset));
    curr_id += timeoffset_I_C->getSize();

    cam0[EST_TIMEOFFSET] = timeoffset_I_C;
  }

  state_[SubStateName::CAM0] = cam0;


  /********************************************************************************/                       
  /********************************** Covariance **********************************/
  /********************************************************************************/

  cov_ = 1e-3 * Eigen::MatrixXd::Identity(curr_id, curr_id);

  /*==============================  DVL ==============================*/

  //// priors for dvl imu extrinsic-rotation calibration
  if(param.msckf.do_R_I_D){
    cov_.block(dvl[EST_QUATERNION]->getId(), dvl[EST_QUATERNION]->getId(), 3, 3) = 
      std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
  }

  //// priors for dvl imu extrinsic-translation calibration
  if(param.msckf.do_p_I_D) {
    cov_.block(dvl[EST_POSITION]->getId(), dvl[EST_POSITION]->getId(), 3, 3) =
      std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
  }
  
  //// priors for dvl imu timeoffset calibration
  if(param.msckf.do_time_I_D) {
    cov_(dvl[EST_TIMEOFFSET]->getId(), dvl[EST_TIMEOFFSET]->getId()) = std::pow(0.01, 2);
  }

  //// priors for dvl scale calibration
  if(param.msckf.do_scale_D) {
    cov_(dvl[EST_SCALE]->getId(), dvl[EST_SCALE]->getId()) = std::pow(0.01, 2);
  }

  /*==============================  CAM0 ==============================*/

  //// priors for cam0 imu extrinsic-rotation calibration
  if(param.msckf.do_R_I_C){
    cov_.block(cam0[EST_QUATERNION]->getId(), cam0[EST_QUATERNION]->getId(), 3, 3) = 
      std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
  }

  //// priors for cam0 imu extrinsic-translation calibration
  if(param.msckf.do_p_I_C) {
    cov_.block(cam0[EST_POSITION]->getId(), cam0[EST_POSITION]->getId(), 3, 3) =
      std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
  }
  
  //// priors for cam0 imu timeoffset calibration
  if(param.msckf.do_time_I_C) {
    cov_(cam0[EST_TIMEOFFSET]->getId(), cam0[EST_TIMEOFFSET]->getId()) = std::pow(0.01, 2);
  }

}

}