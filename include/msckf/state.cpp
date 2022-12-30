#include "state.h"

namespace msckf_dvio {

State::State(const Params &param) {

  params_msckf_ = param.msckf;

  /********************************************************************************/                       
  /************************************ State *************************************/
  /********************************************************************************/
  int curr_id = 0;

  for(const auto& sensor : param.sys.sensors) {

    switch(sensor) {
      case Sensor::IMU: {

        // state

        // rotation
        auto rotation_I_G = std::make_shared<QuatJPL>();
        rotation_I_G->setId(curr_id);
        curr_id += rotation_I_G->getSize();
        state_[Sensor::IMU].emplace(EST_QUATERNION, rotation_I_G);

        // position
        auto position_G_I = std::make_shared<Vec>(3);
        position_G_I->setId(curr_id);
        curr_id += position_G_I->getSize();
        state_[Sensor::IMU].emplace(EST_POSITION, position_G_I);

        // velocity
        auto velocity_G_I = std::make_shared<Vec>(3);
        velocity_G_I->setId(curr_id);
        curr_id += velocity_G_I->getSize();
        state_[Sensor::IMU].emplace(EST_VELOCITY, velocity_G_I);

        // bias gyro
        auto bias_gyro = std::make_shared<Vec>(3);
        bias_gyro->setId(curr_id);
        curr_id += bias_gyro->getSize();
        state_[Sensor::IMU].emplace(EST_BIAS_G, bias_gyro);

        // bias acce
        auto bias_acce = std::make_shared<Vec>(3);
        bias_acce->setId(curr_id);
        curr_id += bias_acce->getSize();
        state_[Sensor::IMU].emplace(EST_BIAS_A, bias_acce);

        // covariance
        cov_ = 1e-3 * Eigen::MatrixXd::Identity(curr_id, curr_id);

        break;
      }

      case Sensor::DVL: {
        // extrinsic_rotation
        if(param.msckf.do_R_I_D){
          // state
          auto extrinsic_q_I_D = std::make_shared<QuatJPL>();
          extrinsic_q_I_D->setId(curr_id);
          extrinsic_q_I_D->setValue(param.prior_dvl.extrinsics.head(4));
          curr_id += extrinsic_q_I_D->getSize();
          state_[Sensor::DVL].emplace(EST_QUATERNION, extrinsic_q_I_D);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::DVL].at(EST_QUATERNION)->getId();
          cov_.block(id, id, 3, 3) = std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);          
        }

        // extrinsic_position: 
        if(param.msckf.do_p_I_D) {
          // state
          auto extrinsic_p_I_D = std::make_shared<Vec>(3);
          extrinsic_p_I_D->setId(curr_id);
          extrinsic_p_I_D->setValue(param.prior_dvl.extrinsics.tail(3));
          curr_id += extrinsic_p_I_D->getSize();
          state_[Sensor::DVL].emplace(EST_POSITION, extrinsic_p_I_D);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::DVL].at(EST_POSITION)->getId();
          cov_.block(id, id, 3, 3) = std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);          
        }

        // timeoffset: t_imu = t_dvl + t_offset
        if(param.msckf.do_time_I_D) {
          // state
          auto timeoffset_I_D = std::make_shared<Vec>(1);
          timeoffset_I_D->setId(curr_id);
          timeoffset_I_D->setValue(Eigen::MatrixXd::Constant(1,1,param.prior_dvl.timeoffset));
          curr_id += timeoffset_I_D->getSize();
          state_[Sensor::DVL].emplace(EST_TIMEOFFSET, timeoffset_I_D);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::DVL].at(EST_TIMEOFFSET)->getId();
          cov_(id, id) = std::pow(0.01, 2);          
        }

        // scale: "Scale"
        if(param.msckf.do_scale_D) {
          // state
          auto scale_D = std::make_shared<Vec>(1);
          scale_D->setId(curr_id);
          scale_D->setValue(Eigen::MatrixXd::Constant(1,1,param.prior_dvl.scale));
          curr_id += scale_D->getSize();
          state_[Sensor::DVL].emplace(EST_SCALE, scale_D);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::DVL].at(EST_SCALE)->getId();
          cov_(id, id) = std::pow(0.01, 2);          
        }

        break;
      }

      case Sensor::PRESSURE: {
        break;
      }

      case Sensor::CAM0: {
        // extrinsic_rotation
        if(param.msckf.do_R_C_I) {
          // state
          auto extrinsic_q_C_I = std::make_shared<QuatJPL>();
          extrinsic_q_C_I->setId(curr_id);
          extrinsic_q_C_I->setValue(param.prior_cam.extrinsics.head(4));
          curr_id += extrinsic_q_C_I->getSize();
          state_[Sensor::CAM0].emplace(EST_QUATERNION, extrinsic_q_C_I);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::CAM0].at(EST_QUATERNION)->getId();
          cov_.block(id, id, 3, 3) = std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);   
        }

        // extrinsic_position: 
        if(param.msckf.do_p_C_I) {
          // state
          auto extrinsic_p_C_I = std::make_shared<Vec>(3);
          extrinsic_p_C_I->setId(curr_id);
          extrinsic_p_C_I->setValue(param.prior_cam.extrinsics.tail(3));
          curr_id += extrinsic_p_C_I->getSize();
          state_[Sensor::CAM0].emplace(EST_POSITION, extrinsic_p_C_I);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::CAM0].at(EST_POSITION)->getId();
          cov_.block(id, id, 3, 3) = std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3); 
        }

        // timeoffset: t_imu = t_cam + t_offset
        if(param.msckf.do_time_C_I) {
          // state
          auto timeoffset_C_I = std::make_shared<Vec>(1);
          timeoffset_C_I->setId(curr_id);
          timeoffset_C_I->setValue(Eigen::MatrixXd::Constant(1,1,param.prior_cam.timeoffset));
          curr_id += timeoffset_C_I->getSize();
          state_[Sensor::CAM0].emplace(EST_TIMEOFFSET, timeoffset_C_I);

          // covariance
          cov_.conservativeResize(curr_id,curr_id);
          auto id = state_[Sensor::CAM0].at(EST_TIMEOFFSET)->getId();
          cov_(id, id) = std::pow(0.01, 2);  
        }

        break;
      }

      default:
        break;
    }
  }

}

double State::getMarginalizedTime(const Sensor sub_state_name) {
  double time = INFINITY;
  for (const auto &clone : state_[sub_state_name]) {
    double clone_time = std::stod(clone.first);
    if (clone_time < time) {
      time = clone_time;
    }
  }
  return time;
}

double State::getCloneTime(const Sensor sub_state_name, int index) {

  // select clone based on given index
  auto iter = std::next(state_[sub_state_name].begin(),index);

  // select clone timestamp and convert to double type
  return std::stod(iter->first);
}

Eigen::VectorXd State::getClonePose(const Sensor sub_state_name, int index) {
  // select clone based on given index
  auto iter = std::next(state_[sub_state_name].begin(),index);

  // select clone pose (vectorXd type)
  return iter->second->getValue();
}

}