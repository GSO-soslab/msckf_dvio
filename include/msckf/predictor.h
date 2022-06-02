#ifndef MSCKF_MSCKF_PREDICTOR_H
#define MSCKF_MSCKF_PREDICTOR_H

#include "types/type_all.h"
#include "utils/utils.h"
#include "msckf/state.h"


namespace msckf_dvio
{

class Predictor {

public:
  Predictor(priorImu prior_imu);

  void propagate(std::shared_ptr<State> state, const std::vector<ImuMsg> &data);

  void augmentDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w);

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

  /// prior information for IMU
  priorImu prior_imu_;

};

} // namespace msckf_dvio


#endif // MSCKF_MSCKF_PREDICTOR_H