#ifndef MSCKF_TYPE_VARIABLES_H_
#define MSCKF_TYPE_VARIABLES_H_

#include <Eigen/Eigen>
#include "utils/utils.h"

namespace msckf_dvio {

typedef long long int StateID;

/************************************************************/
/************************* Vector 3d ************************/
/************************************************************/
class Vec {

public:
  Vec() {
    //// set our default state value
    Eigen::VectorXd vec_default = Eigen::VectorXd::Zero(3, 1);
    setValue(vec_default);
  }

  ~Vec() {}

  /******************** Set variables ********************/

  void setValue(const Eigen::MatrixXd &new_value) {
    assert(new_value.rows() == 3);
    assert(new_value.cols() == 1);
    //// set value
    value_ = new_value;
  }

  void setCoordinate(int new_Coordinate) { coordinate_ = new_Coordinate; }

  /******************** get variables ********************/

  Eigen::MatrixXd value() const { return value_; }

  int coordinate() { return coordinate_; }

private:
  //// current best estimate 
  Eigen::VectorXd value_;
  //// Location of error state in covariance
  int coordinate_ = -1;
};


/************************************************************/
/*************************** Pose ***************************/
/************************************************************/

class PoseJPL {

public:
  PoseJPL() {
    //// set our default state value
    Eigen::VectorXd pose_default = Eigen::VectorXd::Zero(7, 1);
    pose_default(3) = 1.0;
    setValue(pose_default);
  }

  ~PoseJPL() {}

  /******************** Set variables ********************/

  void setValue(const Eigen::MatrixXd &new_value) {
    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    //// set orientation value
    q_ = new_value.block(0, 0, 4, 1);
    //// set position value
    p_ = new_value.block(4, 0, 3, 1);
    //// set value
    value_ = new_value;
  }

  void setCoordinate(int new_Coordinate) { coordinate_ = new_Coordinate; }

  /******************** Get variables ********************/

  Eigen::Vector4d quaternion() const { return q_; }

  Eigen::Vector3d position() const { return p_; }

  Eigen::Matrix3d rotation() { 
    return toRotationMatrix(q_);
  } 

  Eigen::Matrix4d transformation() { 
    Eigen::Matrix4d trans;
    trans.block<3,3>(0,0) = toRotationMatrix(q_);
    trans.block<3,1>(0,3) = p_;
    return trans;
  } 

  int coordinate() { return coordinate_; }


private:
  //// JPL quaternion
  Eigen::Vector4d q_;
  //// position
  Eigen::Vector3d p_;
  //// current best estimate 
  Eigen::VectorXd value_;
  //// Location of error state in covariance
  int coordinate_ = -1;
};

/************************************************************/
/**************************** IMU ***************************/
/************************************************************/

class Imu {

public:
  Imu() {
    // Create all the sub-variables
    pose_ = std::shared_ptr<PoseJPL>(new PoseJPL());
    v_ = std::shared_ptr<Vec>(new Vec());
    bg_ = std::shared_ptr<Vec>(new Vec());
    ba_ = std::shared_ptr<Vec>(new Vec());

    // Set our default state value
    Eigen::VectorXd imu_default = Eigen::VectorXd::Zero(16, 1);
    imu_default(3) = 1.0;
    setValue(imu_default);

  }

  ~Imu() {}

  /******************** Set variables ********************/

  void setValue(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 16);
    assert(new_value.cols() == 1);

    pose_->setValue(new_value.block(0, 0, 7, 1));
    v_->setValue(new_value.block(7, 0, 3, 1));
    bg_->setValue(new_value.block(10, 0, 3, 1));
    ba_->setValue(new_value.block(13, 0, 3, 1));

    value_ = new_value;
  }

  void setCoordinate(int new_Coordinate) { coordinate_ = new_Coordinate; }


  /******************** Get variables ********************/

  Eigen::Vector4d quaternion() const { return pose_->quaternion(); }

  Eigen::Vector3d position() const { return pose_->position(); }

  Eigen::Matrix3d rotation() { 
    return toRotationMatrix(pose_->quaternion());
  } 

  Eigen::Matrix4d transformation() { 
    Eigen::Matrix4d trans;
    trans.block<3,3>(0,0) = toRotationMatrix(pose_->quaternion());
    trans.block<3,1>(0,3) = pose_->position();
    return trans;
  } 

  Eigen::Vector3d velocity() const { return v_->value(); }

  Eigen::Vector3d biasG() const { return bg_->value(); }

  Eigen::Vector3d biasA() const { return bg_->value(); }

  int coordinate() { return coordinate_; }

private:
  // // specific time for this IMU state 
  // double time;
  // // specific id for this IMU state  
  // StateID id;  
  
  //// Pose subvariable
  std::shared_ptr<PoseJPL> pose_;
  //// Velocity subvariable
  std::shared_ptr<Vec> v_;
  //// Gyroscope bias subvariable
  std::shared_ptr<Vec> bg_;
  //// Acceleration bias subvariable
  std::shared_ptr<Vec> ba_;

  //// current best estimate 
  Eigen::VectorXd value_;
  //// Location of error state in covariance
  int coordinate_ = -1;
};




} // namespace msckf_dvio

#endif  //MSCKF_TYPE_VARIABLES_H_