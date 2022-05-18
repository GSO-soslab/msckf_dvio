#ifndef MSCKF_TYPE_H_
#define MSCKF_TYPE_H_

#include <Eigen/Eigen>
#include <memory>

namespace msckf_dvio {

class Type
{

public:
  
  //! @brief Default constructor for our Type
  //! @param new_size: degrees of freedom of variable (i.e., the size of the error state)
  Type(int new_size) { size_ = new_size; }

  virtual ~Type(){};

  //! @brief Sets id used to track location of variable in the filter covariance
  //! @note Note that the minimum ID is -1 which says that the state is not in our covariance.
  //!       If the ID is larger than -1 then this is the index location in the covariance matrix.
  //! @param new_id: entry in filter covariance corresponding to this variable
  virtual void setId(int new_id) { id_ = new_id; }

  //! @brief Overwrite value of state's estimate
  //! @param new_value New value that will overwrite state's value
  virtual void setValue(const Eigen::MatrixXd &new_value) {
    assert(value_.rows() == new_value.rows());
    assert(value_.cols() == new_value.cols());
    value_ = new_value;
  }

  //! @brief Overwrite value of first-estimate
  //! @param new_value New value that will overwrite state's fej
  virtual void setFej(const Eigen::MatrixXd &new_value) {
    assert(fej_.rows() == new_value.rows());
    assert(fej_.cols() == new_value.cols());
    fej_ = new_value;
  }

  //! @brief Access to variable id (i.e. its location in the covariance)
  const int getId() const { return id_; }

  //! @brief Access variable's estimate
  virtual const Eigen::MatrixXd &getValue() const { return value_; }

  //! @brief Access variable's first-estimate
  virtual const Eigen::MatrixXd &getFej() const { return fej_; }

  //! @brief Access to variable size (i.e. its error state size)
  int getSize() { return size_; }

  //! @brief Update variable due to perturbation of error state
  //! @param dx Perturbation used to update the variable through a defined "boxplus" operation
  virtual void update(const Eigen::VectorXd &dx) = 0;

  //! @brief Create a clone of this variable
  virtual std::shared_ptr<Type> clone() = 0;

protected:
  /// First-estimate
  Eigen::MatrixXd fej_;

  /// Current best estimate
  Eigen::MatrixXd value_;

  /// state location
  int id_ = -1;

  /// Dimension of error state
  int size_ = -1;
};

} // namespace msckf_dvio

#endif  //MSCKF_TYPE_H_