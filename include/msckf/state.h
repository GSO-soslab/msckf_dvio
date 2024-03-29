#ifndef MSCKF_MSCKF_STATE_H_
#define MSCKF_MSCKF_STATE_H_

#include <memory>
#include <map>
#include <string>
#include <iostream>

#include "types/type.h"
#include "types/vec.h"
#include "types/quatJPL.h"

// #include "msgs.h"
#include "types/parameters.h"

namespace msckf_dvio
{

//! @brief state for each sensor and clone, e.g. IMU, CAM, DVL, CLONE_CAM ...
//!
typedef std::map<std::string, std::shared_ptr<Type>> SubState;

//! @brief full state including all the sub state
//!
typedef std::map<Sensor, SubState> FullState;


class State {
  
public:
  State(const Params &param);

  ~State() {}

  inline double getTimestamp() const { return timestamp_; }

  inline void setTimestamp(const double t) { timestamp_ = t; }

  /********************************************************************************/                       
  /************************************** State ***********************************/
  /********************************************************************************/
  //! @brief get each sub state including several actual estimated states, e.g. rotation, position
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  inline SubState getSubState(const Sensor sub_state_name) { return state_[sub_state_name]; }
 
  //! @brief get entire estimated state
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rotation, position, calibration... 
  inline std::shared_ptr<Type> getEstimation(const Sensor sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name);
  }

  //! @brief get each estimated state value
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rotation, position, calibration...
  inline Eigen::VectorXd getEstimationValue(const Sensor sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name)->getValue();
  }

  //! @brief get each estimated state location in the state vector or covariance
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rotation, position, calibration...
  inline int getEstimationId(const Sensor sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name)->getId();
  }

  //! @brief get each estimated state size
  //!
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rotation, position, calibration...
  //!
  inline int getEstimationSize(const Sensor sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name)->getSize();
  }

  //! @brief how many clones in CLONE STATE
  //!
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //!
  inline int getEstimationNum(const Sensor sub_state_name) {
    return state_[sub_state_name].size();
  }

  //! @brief the timestamp of pose that will marginalize next, the earliest time  
  //!
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //!
  double getMarginalizedTime(const Sensor sub_state_name);

  //! @brief the timestamp of clone with given index
  //!
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param index: index of marg clone from window
  //!
  double getCloneTime(const Sensor sub_state_name, int index);

  //! @brief the clone pose with given index
  //!
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param index: index of marg clone from window
  //!
  Eigen::VectorXd getClonePose(const Sensor sub_state_name, int index);

  //! @brief set value of angle estimation of any sub state, will override the original value
  //!
  //! @param sub_state_name: sub state name, e.g. IMU, DVL, CAM, CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rotation, position, calibration...
  //! @param new_value: estimated value
  //!
  inline void setEstimationValue(const Sensor &sub_state_name, const std::string &est_name, const Eigen::MatrixXd &new_value){
    state_[sub_state_name].at(est_name)->setValue(new_value);
  }

  //! @brief update state using estimated state error
  //!
  //! @param new_value: estimated state error
  //!
  inline void updateState(const Eigen::VectorXd &new_value) {
    assert(new_value.rows() == cov_.rows());

    // find each sub states: IMU, DVL, CLONE
    for(auto & sub_state: state_) {
      // find each estimation: q, p, v, bias, timeoffset, clone_pose ....
      for(auto &estimation : sub_state.second) {
        // grab associated block based on state id and state size
        estimation.second->update(new_value.block(estimation.second->getId(),   0, 
                                                  estimation.second->getSize(), 1));
      }
    }
  }

  void testClone(const Sensor sub_state_name) {
    for(const auto & clone : state_[sub_state_name]) {
      printf("  t: %s\n", clone.first.c_str());
    }
  }

  //==================================== IMU =====================================//


  inline Eigen::VectorXd getImuValue() { 
    Eigen::Matrix<double, 16, 1> imu_value;
    imu_value.block(0, 0, 4, 1)  = state_[IMU].at(EST_QUATERNION)->getValue();
    imu_value.block(4, 0, 3, 1)  = state_[IMU].at(EST_POSITION)->getValue();
    imu_value.block(7, 0, 3, 1)  = state_[IMU].at(EST_VELOCITY)->getValue();
    imu_value.block(10, 0, 3, 1) = state_[IMU].at(EST_BIAS_G)->getValue();
    imu_value.block(13, 0, 3, 1) = state_[IMU].at(EST_BIAS_A)->getValue();
    return imu_value;
  }

  inline void setImuValue(const Eigen::Matrix<double, 16, 1> &new_value) {
    state_[IMU].at(EST_QUATERNION)->setValue(new_value.block(0, 0, 4, 1));
    state_[IMU].at(EST_POSITION)->setValue(new_value.block(4, 0, 3, 1));
    state_[IMU].at(EST_VELOCITY)->setValue(new_value.block(7, 0, 3, 1));
    state_[IMU].at(EST_BIAS_G)->setValue(new_value.block(10, 0, 3, 1));
    state_[IMU].at(EST_BIAS_A)->setValue(new_value.block(13, 0, 3, 1));
  }

  inline int getImuSize() {
    int size = 0;
    size += state_[IMU].at(EST_QUATERNION)->getSize();
    size += state_[IMU].at(EST_POSITION)->getSize();
    size += state_[IMU].at(EST_VELOCITY)->getSize();
    size += state_[IMU].at(EST_BIAS_G)->getSize();
    size += state_[IMU].at(EST_BIAS_A)->getSize();
    return size;
  }

  //================================= CLONE ========================================//

  inline bool foundClone(const Sensor &sub_state_name, const std::string &clone_name) {
    return state_[sub_state_name].find(clone_name) != state_[sub_state_name].end() ? true : false; 
  }
  
  //================================= Pressure ========================================//

  inline void setPressureInit(double value) {pressure_init = value;}

  inline double getPressureInit() {return pressure_init;}
  /********************************************************************************/                       
  /********************************** Covariance **********************************/
  /********************************************************************************/

  inline Eigen::MatrixXd getCov() const { return cov_; }

  inline int getCovRows()  const { return cov_.rows(); }

  inline int getCovCols()  const { return cov_.cols(); }

  //! @brief: We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d) 
  //!
  inline bool foundSPD(const std::string &mission) {
    Eigen::VectorXd diags = cov_.diagonal();

    for (int i = 0; i < diags.rows(); i++) {
      if (diags(i) < 0.0) {
        printf("Covariance %s: - diagonal at %d is %.9f\n", mission.c_str(), i+1, diags(i));
        return true;
      }
    }

    return false;
  }

  Eigen::MatrixXd getPartCov(const std::vector<std::shared_ptr<Type>> &state_order);

private:
  friend class Predictor;

  friend class Updater;

  // Current timestamp (should be the last update time!)
  double timestamp_;

  // state vector
  FullState state_;

  // covariance
  Eigen::MatrixXd cov_;

  // parameters for MSCKF operation
  paramMsckf params_msckf_;

  // other sensor propetry
  double pressure_init;
};
  
} // namespace msckf_dvio


#endif //MSCKF_MSCKF_UPDATER_H