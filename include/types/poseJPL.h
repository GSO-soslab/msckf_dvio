#ifndef MSCKF_TYPE_POSE_JPL_H_
#define MSCKF_TYPE_POSE_JPL_H_

#include "type.h"
#include "utils/utils.h"

namespace msckf_dvio {

class PoseJPL : public Type
{
public:
  PoseJPL() : Type(6) {
    Eigen::MatrixXd pose0 = Eigen::MatrixXd::Zero(7,1);
    pose0(3) = 1.0;
    setValue(pose0);
    setFej(pose0);
  }

  ~PoseJPL() {}

  void setValue(const Eigen::MatrixXd &new_value) override {
    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    value_ = new_value;
  }

  void setFej(const Eigen::MatrixXd &new_value) override { 
    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    fej_ = new_value;  
  }

  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == size_);

    //  quaternion
    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx.block(0,0,4,1), 1.0;
    dq = normalizeQuat(dq);

    // Update 
    Eigen::Matrix<double, 7, 1> pose;
    pose.block(0,0,4,1) = multiplyQuat(dq, value_.block(0,0,4,1));
    pose.block(4,0,3,1) = value_.block(4,0,3,1) + dx.block(4,0,3,1);
    setValue(pose);
  }

  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<PoseJPL>(new PoseJPL());
    Clone->setValue(getValue());
    Clone->setFej(getFej());
    return Clone;
  }

protected:

};

} // namespace msckf_dvio

#endif  //MSCKF_TYPE_POSE_JPL_H_