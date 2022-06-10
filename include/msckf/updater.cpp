#include "updater.h"

namespace msckf_dvio
{

Updater::Updater(priorDvl prior_dvl, paramMsckf param_msckf) : 
  prior_dvl_(prior_dvl), param_msckf_(param_msckf)
  {}

void Updater::updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D) {
  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  // temp values
  double scale = state->getEstimationValue(DVL,EST_SCALE)(0);
  Eigen::Matrix3d R_I_G = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION));
  Eigen::Vector3d p_I_D = state->getEstimationValue(DVL,EST_POSITION);

  // temp 1: 1/S * R_I_D^T
  Eigen::Matrix3d temp_1 = 1 / scale * toRotationMatrix(state->getEstimationValue(DVL,EST_QUATERNION)).transpose();
  // temp 2: R_I_G * v_G_I
  Eigen::Vector3d temp_2 = R_I_G * state->getEstimationValue(IMU,EST_VELOCITY);
  // temp 3: R_I_G * v_G_I + [w_I]x * p_I_D
  Eigen::Vector3d temp_3 = temp_2 + toSkewSymmetric(w_I) * p_I_D;
                          
  // dh()/d(imu_R_I_G) - (3x3): 1/S * R_I_D^T * [R_I_G * v_G_I]x
  Eigen::Matrix3d d_imu_r = temp_1 * toSkewSymmetric(temp_2);
  // dh()/d(imu_p_G_I) - (3x3): 0
  // dh()/d(imu_v_G_I) - (3x3): 1/S * R_I_D^T * R_I_G
  Eigen::Matrix3d d_imu_v = temp_1 * R_I_G;
  // dh()/d(imu_bw)    - (3x3): - 1/S * R_I_D^T * [p_I_D]x ??????
  // dh()/d(imu_ba)    - (3x3): 0

  // dh()/d(dvl_R_I_D) - (3x3):  - 1/S * R_I_D^T * [R_I_G * v_G_I + [w_I]x * p_I_D]x
  Eigen::Matrix3d d_dvl_r = - temp_1 * toSkewSymmetric(temp_3);
  // dh()/d(dvl_p_I_D) - (3x3):  1/S * R_I_D^T * [w_I]x
  Eigen::Matrix3d d_dvl_p = temp_1 * toSkewSymmetric(w_I);
  // dh()/d(dvl_t_I_D) - (3x1):  ??????
  // dh()/d(dvl_scale) - (3x1):  - 1/S^2 * R_I_D^T * (R_I_G * v_G_I + [w_I]x * p_I_D)
  Eigen::Vector3d d_dvl_s = - temp_1 / scale * temp_3;

  //! TODO: if tf, time, scale are not calibrated, the Jacobian function will different
  // H = [H_imu(3x15) H_transform(3x6) H_time(3x1) H_scale(3x1) H_clone(3x6N)]
  auto id_imu_r = state->getEstimationId(IMU,EST_QUATERNION);
  auto id_imu_v = state->getEstimationId(IMU,EST_VELOCITY);
  auto id_dvl_r = state->getEstimationId(DVL,EST_QUATERNION);
  auto id_dvl_p = state->getEstimationId(DVL,EST_POSITION);
  auto id_dvl_s = state->getEstimationId(DVL,EST_SCALE);

  int size_measurement = 3;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  H.block(0,id_imu_r,3,3) = d_imu_r;
  H.block(0,id_imu_v,3,3) = d_imu_v;
  H.block(0,id_dvl_r,3,3) = d_dvl_r;
  H.block(0,id_dvl_p,3,3) = d_dvl_p;
  H.block(0,id_dvl_s,3,1) = d_dvl_s;

  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/
  // K = P * H^T * (H * P * H^T + Rn) ^-1

  // DVL BT measurement noise matrix
  Eigen::Matrix3d Rn = Eigen::Matrix3d::Identity();
  Rn(0,0) = pow(prior_dvl_.sigma_bt(0), 2);
  Rn(1,1) = pow(prior_dvl_.sigma_bt(1), 2);
  Rn(2,2) = pow(prior_dvl_.sigma_bt(2), 2);

  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  // S.triangularView<Eigen::Upper>() = H * state->cov_ * H.transpose();
  // S.triangularView<Eigen::Upper>() += Rn;
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();

  std::cout<<"H: "<< H<<std::endl;
  std::cout<<"K: "<< K<<std::endl;
  // std::cout<<"delta X: "<< delta_X.transpose()<<std::endl;

  /********************************************************************************/
  /*************************** Update state and covariance ************************/
  /********************************************************************************/

  // DVL BT estimation: z = 1/S * R_I_D^T * (R_I_G * v_G_I + [w_I]x * p_I_D) + n
  Eigen::Vector3d v_D_hat = temp_1 * temp_3;
  Eigen::Vector3d r = v_D - v_D_hat;
  Eigen::VectorXd delta_X = K * r;

  state->updateState(delta_X);

  // P_k = P_k-1 - K * H * P_k-1
  state->cov_.triangularView<Eigen::Upper>() -= K * H * state->cov_;
  state->cov_ = state->cov_.selfadjointView<Eigen::Upper>();
}

void Updater::updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D, bool is_simple) {
  // std::ofstream file;

  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/
  int size_measurement = 3;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  auto id_imu_v = state->getEstimationId(IMU,EST_VELOCITY);

  H.block(0,id_imu_v,3,3) = Eigen::Matrix3d::Identity();
  // std::cout<<"H: "<< H<<std::endl;

  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/
  // IMU veloicty measurement noise
  Eigen::Matrix3d Rn = Eigen::Matrix3d::Identity();
  Rn(0,0) = pow(0.1, 2);
  Rn(1,1) = pow(0.1, 2);
  Rn(2,2) = pow(0.1, 2);

 // K = P * H^T * (H * P * H^T + Rn) ^-1
  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();
  // std::cout<<"K1: "<< K<<std::endl;

  Eigen::Vector3d p_I_D = prior_dvl_.extrinsics.tail(3);
  Eigen::Vector3d temp = toSkewSymmetric(w_I)*p_I_D;

  Eigen::Vector3d v_I_meas,v_I_meas_2;
  // v_I_meas  = v_D;

  v_I_meas << -v_D(0), v_D(1), -v_D(2);
  // v_I_meas = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION))* (v_I_meas * (1417/1500.0) - temp);
  v_I_meas = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION)).transpose() * (v_I_meas * (1417/1500.0) - temp);

  //// v_measurement_1; v_measurement_2; v_before_update; v_after_update
  // file.open(file_path, std::ios_base::app);//std::ios_base::app
  // file<< v_I_meas(0)<<","<<v_I_meas(1)<<","<<v_I_meas(2)<<"; ";
  // file<< v_I_meas_2(0)<<","<<v_I_meas_2(1)<<","<<v_I_meas_2(2)<<"; ";

  Eigen::Vector3d r = v_I_meas - state->getEstimationValue(IMU,EST_VELOCITY);
  Eigen::VectorXd delta_X = K * r;

  //update state
  // Eigen::Vector3d v_bef = state->getEstimationValue(IMU,EST_VELOCITY);
  // file<< v_bef(0)<<","<<v_bef(1)<<","<<v_bef(2)<<"; ";
  state->updateState(delta_X);
  // Eigen::Vector3d v_aft = state->getEstimationValue(IMU,EST_VELOCITY);
  // file<< v_aft(0)<<","<<v_aft(1)<<","<<v_aft(2)<<"\n";
  // file.close();

  //update covariance
  // P_k = P_k-1 - K * H * P_k-1

  Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(K.rows(), H.cols()) - K*H;
  state->cov_ = I_KH*state->cov_;

  // Fix the covariance to be symmetric
  Eigen::MatrixXd state_cov_fixed = (state->cov_ + state->cov_.transpose()) / 2.0;
  state->cov_ = state_cov_fixed;
}

void Updater::marginalizeDvl(std::shared_ptr<State> state) {

  /****************************************************************/
  /******************** marginalize covariance ********************/
  /****************************************************************/
  
  // find the clone need to marginalize
  auto marg = state->state_[CLONE_DVL].begin();

  auto time_marg = marg->first;
  auto id_marg = marg->second->getId();
  auto size_marg = marg->second->getSize();
  auto size_marg_bef = marg->second->getId();
  auto size_marg_aft = (int)state->cov_.rows() - size_marg_bef - size_marg;

  //  P_(x_bef,x_bef)  P(x_bef,x_marg)  P(x_bef,x_aft)
  //  P_(x_marg,x_bef) P(x_marg,x_marg) P(x_marg,x_aft)
  //  P_(x_aft,x_bef)  P(x_aft,x_marg)  P(x_aft,x_aft)
  //  
  //  to
  //
  //  P_(x_bef,x_bef) P(x_bef,x_aft)
  //  P_(x_aft,x_bef) P(x_aft,x_aft)

  // remove covariance
  Eigen::MatrixXd cov_new(state->cov_.rows() - size_marg, state->cov_.rows() - size_marg);

  // P_(x_bef,x_bef)
  cov_new.block(0, 0, size_marg_bef, size_marg_bef) = 
    state->cov_.block(0, 0, size_marg_bef, size_marg_bef);
  // P_(x_bef,x_aft)
  cov_new.block(0, size_marg_bef, size_marg_bef, size_marg_aft) = 
    state->cov_.block(0, size_marg_bef + size_marg, size_marg_bef, size_marg_aft);
  // P_(x_aft,x_bef)
  cov_new.block(size_marg_bef, 0, size_marg_aft, size_marg_bef) = 
    cov_new.block(0, size_marg_bef, size_marg_bef, size_marg_aft).transpose();
  // P_(x_aft,x_aft)
  cov_new.block(size_marg_bef, size_marg_bef, size_marg_aft, size_marg_aft) = 
    state->cov_.block(size_marg_bef + size_marg, size_marg_bef + size_marg, size_marg_aft, size_marg_aft);

  // set to new covariance
  state->cov_ = cov_new;
  assert(state->cov_.rows() == cov_new.rows());
  assert(state->cov_.cols() == cov_new.cols());

  /****************************************************************/
  /********************** marginalize clone ***********************/
  /****************************************************************/

  // remove clone
  //! TODO: make this as function with clone_type and time_marg, could also used for camera marg
  state->state_[CLONE_DVL].erase(time_marg);

  // reset id
  for(auto &estimation : state->state_[CLONE_DVL]) {
    auto id = estimation.second->getId();

    if( id > id_marg) 
      estimation.second->setId(id - size_marg);
  }
}

} // namespace msckf_dvio