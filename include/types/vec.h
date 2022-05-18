#ifndef MSCKF_TYPE_VEC_H_
#define MSCKF_TYPE_VEC_H_

#include "type.h"

namespace msckf_dvio {

//! @brief Derived Type class that implements vector variables
class Vec : public Type
{
public:
  //! @brief Default constructor for Vec
  //! @param dim: Size of the vector (will be same as error state)
  Vec(int dim) : Type(dim) {
    value_ = Eigen::VectorXd::Zero(dim);
    fej_ = Eigen::VectorXd::Zero(dim);
  }

  ~Vec() {}

  //! @brief Implements the update operation through standard vector addition
  //! @param dx Additive error state correction
  void update(const Eigen::VectorXd &dx) override {
    assert(dx.rows() == size_);
    setValue(value_ + dx);
  }

  //! @brief Performs all the cloning
  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<Type>(new Vec(size_));
    Clone->setValue(getValue());
    Clone->setFej(getFej());
    return Clone;
  }
};

} // namespace msckf_dvio

#endif  //MSCKF_TYPE_VEC_H_