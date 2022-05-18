#ifndef MSCKF_CORE_PREDICTOR_H
#define MSCKF_CORE_PREDICTOR_H

#include "types/type_all.h"
#include "utils/utils.h"
#include "core/state.h"

#include <mutex>



namespace msckf_dvio
{

class Predictor {

public:
  Predictor(noiseImu noise, double gravity);

  void propagate(std::shared_ptr<State> state, const std::vector<ImuMsg> &data);

  void augment(std::shared_ptr<State> state, const Eigen::Vector3d &w);

private:
  void propagateState(std::shared_ptr<State> state, 
                      const ImuMsg &data_begin, 
                      const ImuMsg &data_end,
                      Eigen::Matrix<double, 15, 15> &F, 
                      Eigen::Matrix<double, 15, 15> &Qd);

  void propagateCovariance(std::shared_ptr<State> state, 
                           const Eigen::MatrixXd &Phi,
                           const Eigen::MatrixXd &Q);

  void predict_mean_rk4(std::shared_ptr<State> state, double dt, 
                        const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                        const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, 
                        Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p);

  /// Container for the noise values
  noiseImu noises_;

  /// Gravity vector
  Eigen::Vector3d gravity_vector_;

};

} // namespace msckf_dvio


#endif //MSCKF_CORE_PREDICTOR_H