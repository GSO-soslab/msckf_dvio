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

//! Single Estimation State name for each Sub State
// 
#define EST_QUATERNION "Quaternion"
#define EST_POSITION "Position"
#define EST_VELOCITY "Velocity"
#define EST_BIAS_G "BiasGyro"
#define EST_BIAS_A "BiasAcce"
#define EST_TIMEOFFSET "Timeoffset"
#define EST_SCALE "Scale"

//! Sub State name for each sensor and clone
// 
enum SubStateName{
  IMU = 0,
  DVL,
  CAM0,
  CLONE_DVL,
  CLONE_CAM0
};

namespace msckf_dvio
{

//! @brief state for each sensor and clone, e.g. IMU, CAM, DVL, CLONE_CAM ...
//!
typedef std::map<std::string, std::shared_ptr<Type>> SubState;

//! @brief full state including all the sub state
//!
typedef std::map<SubStateName, SubState> FullState;



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
  inline SubState getSubState(const SubStateName sub_state_name) { return state_[sub_state_name]; }
 
  //! @brief get each estimated state value
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rotation, position, calibration...
  inline Eigen::VectorXd getEstimationValue(const SubStateName sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name)->getValue();
  }

  //! @brief get each estimated state location in the state vector or covariance
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rostion, position, calibration...
  inline int getEstimationId(const SubStateName sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name)->getId();
  }

  //! @brief get each estimated state size
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //! @param est_name: each actual estimation in the sub state, e.g. rostion, position, calibration...
  inline int getEstimationSize(const SubStateName sub_state_name,  const std::string &est_name) {
    return state_[sub_state_name].at(est_name)->getSize();
  }

  //! @brief how many estimations in SENSOR STATE or CLONE STATE
  //! @param sub_state_name: each sub state, e.g. SENSOR and CLONE
  //!
  inline int getEstimationNum(const SubStateName sub_state_name) {
    return state_[sub_state_name].size();
  }

  //! @brief update state using estimated state error
  //! @param new_value: estimated state error
  //!
  inline void updateState(const Eigen::VectorXd &new_value) {
    assert(new_value.rows() == cov_.rows());

    // find each sub states: IMU, DVL, CLONE
    for(auto & sub_state: state_) {
      // find each estimation: q, p, v, bias, timeoffset ....
      for(auto &estimation : sub_state.second) {
        // grab associated block based on state id and state size
        estimation.second->update(new_value.block(estimation.second->getId(),   0, 
                                                  estimation.second->getSize(), 1));
      }
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


  //==================================== DVL ========================================//
  
  inline void setDvl(const Eigen::MatrixXd &new_value) {
    //! TODO: 
    //! check size with actuall size based on DVL MSCKF state setting
    //! accroding to id to setup values
  }

  inline void setDvlEst(const std::string &est_name, const Eigen::MatrixXd &new_value) {
    state_[DVL].at(est_name)->setValue(new_value);
  }

  //================================= CLONE_DVL ========================================//

  inline bool foundClone(const SubStateName &sub_state_name, const std::string &clone_name) {
    return state_[sub_state_name].find(clone_name) != state_[sub_state_name].end() ? true : false; 
  }
  
  /********************************************************************************/                       
  /********************************** Covariance **********************************/
  /********************************************************************************/

  inline Eigen::MatrixXd getCov() const { return cov_; }

  inline int getCovRows()  const { return cov_.rows(); }

  inline int getCovCols()  const { return cov_.cols(); }

  inline bool foundSPD() {
    Eigen::VectorXd diags = cov_.diagonal();

    for (int i = 0; i < diags.rows(); i++) {
      if (diags(i) < 0.0) {
        printf("Covariance propagation: - diagonal at %d is %.2f\n", i, diags(i));
        return true;
      }
    }

    return false;
  }

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
};
  
} // namespace msckf_dvio


#endif //MSCKF_MSCKF_UPDATER_H