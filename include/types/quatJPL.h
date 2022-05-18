#ifndef MSCKF_TYPE_QUAT_JPL_H_
#define MSCKF_TYPE_QUAT_JPL_H_

#include "type.h"
#include "utils/utils.h"

namespace msckf_dvio {

 //! @brief: Derived Type class that implements JPL quaternion
 //! @note:
 //!  This quaternion uses a left-multiplicative error state and follows the "JPL convention".
 //!  Please checkout our utility functions in the quat_ops.h file.
 //!  We recommend that people new quaternions check out the following resources:
 //!  - http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 //!  - ftp://naif.jpl.nasa.gov/pub/naif/misc/Quaternion_White_Paper/Quaternions_White_Paper.pdf
 //!
class QuatJPL : public Type
{
public:
  QuatJPL() : Type(3) {
    Eigen::Vector4d q0 = Eigen::Vector4d::Zero();
    q0(3) = 1.0;
    setValue(q0);
    setFej(q0);
  }

  ~QuatJPL() {}

  //! @brief Sets the value of the estimate and recomputes the internal rotation matrix
  //! @param new_value New value for the quaternion estimate
  //!
  void setValue(const Eigen::MatrixXd &new_value) override {
    assert(new_value.rows() == 4);
    assert(new_value.cols() == 1);

    value_ = new_value;

    // // compute associated rotation
    // R_ = toRotationMatrix(new_value);
  }

  //! @brief Sets the fej value and recomputes the fej rotation matrix
  //! @param new_value New value for the quaternion estimate
  //!
  void setFej(const Eigen::MatrixXd &new_value) override { 
    assert(new_value.rows() == 4);
    assert(new_value.cols() == 1);

    fej_ = new_value;

    // // compute associated rotation
    // R_fej_ = toRotationMatrix(new_value);  
  }

  // //! @brief Rotation access
  // //!
  // Eigen::Matrix<double, 3, 3> getRot() const { return R_; }

  // //! @brief FEJ Rotation access
  // //!
  // Eigen::Matrix<double, 3, 3> getRotFej() const { return R_fej_; }

  //! @brief Implements update operation by left-multiplying the current
  //! @note  quaternion with a quaternion built from a small axis-angle perturbation.
  //! @param dx Axis-angle representation of the perturbing quaternion
  //! 
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == size_);

    // Build perturbing quaternion
    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx, 1.0;
    dq = normalizeQuat(dq);

    // Update estimate and recompute R
    setValue(multiplyQuat(dq, value_));
  }

  //! @brief clone
  //!
  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<QuatJPL>(new QuatJPL());
    Clone->setValue(getValue());
    Clone->setFej(getFej());
    return Clone;
  }

protected:
  // // Stores the rotation
  // Eigen::Matrix<double, 3, 3> R_;

  // // Stores the first-estimate rotation
  // Eigen::Matrix<double, 3, 3> R_fej_;

};

} // namespace msckf_dvio

#endif  //MSCKF_TYPE_QUAT_JPL_H_