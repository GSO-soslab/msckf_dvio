#include "updater.h"

namespace msckf_dvio
{

Updater::Updater(priorDvl prior_dvl, paramMsckf param_msckf) : 
  prior_dvl_(prior_dvl), param_msckf_(param_msckf), count(0)
  {}

void Updater::updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D) {

  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  int size_measurement = 3;
  int size_state = state->getCovCols();
  // H = [H_imu(3x15) H_transform(3x6) H_time(3x1) H_scale(3x1) H_clone(3x6N)]
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  /*-------------------- elements need for DVL velocity estimation --------------------*/

  Eigen::Matrix3d R_I_G = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION));

  // check if do the DVL extrinsics-rotation online calibration
  Eigen::Matrix3d R_I_D = param_msckf_.do_R_I_D ? 
                          toRotationMatrix(state->getEstimationValue(DVL,EST_QUATERNION)) :
                          toRotationMatrix(prior_dvl_.extrinsics.head(4)); 

  // check if do the DVL extrinsics-position online calibration
  Eigen::Vector3d p_I_D = param_msckf_.do_p_I_D ? 
                          state->getEstimationValue(DVL,EST_POSITION) : 
                          prior_dvl_.extrinsics.tail(3);
                          
  // check if do the DVL scale online calibration
  double scale = param_msckf_.do_scale_D ? 
                 state->getEstimationValue(DVL,EST_SCALE)(0) : 
                 prior_dvl_.scale;

  // measurement function:
  // DVL BT velocity estimation: v_D = 1/S * R_I_D^T * (R_I_G * v_G_I + [w_I]x * p_I_D) + n

  // temp 1: 1/S * R_I_D^T
  Eigen::Matrix3d temp_1 = 1 / scale * R_I_D.transpose();
  // temp 2: R_I_G * v_G_I
  Eigen::Vector3d temp_2 = R_I_G * state->getEstimationValue(IMU,EST_VELOCITY);
  // temp 3: R_I_G * v_G_I + [w_I]x * p_I_D
  Eigen::Vector3d temp_3 = temp_2 + toSkewSymmetric(w_I) * p_I_D;
                          
  /*-------------------- Jacobian related to IMU --------------------*/

  //! TODO: if R_I_D is not calibrated, this update will cased bad state estimation
  //! TODO: try this if well calibrated
  if(param_msckf_.do_R_I_D){
    // State R_I_G(3x3): dh()/d(imu_R_I_G): 1/S * R_I_D^T * [R_I_G * v_G_I]x
    Eigen::Matrix3d d_imu_r = temp_1 * toSkewSymmetric(temp_2);
    // id 
    auto id_imu_r = state->getEstimationId(IMU,EST_QUATERNION);
    // stack jacobian matrix
    H.block(0,id_imu_r,3,3) = d_imu_r;
  }

  // State p_G_I(3x3): dh()/d(imu_p_G_I): 0

  // State v_G_I(3x3): dh()/d(imu_v_G_I): 1/S * R_I_D^T * R_I_G
  Eigen::Matrix3d d_imu_v = temp_1 * R_I_G;
  // id
  auto id_imu_v = state->getEstimationId(IMU,EST_VELOCITY);
  // stack jacobian matrix
  H.block(0,id_imu_v,3,3) = d_imu_v;

  // State bias_w(3x3): dh()/d(imu_bw): - 1/S * R_I_D^T * [p_I_D]x ??????

  // State bias_a(3x3): dh()/d(imu_ba): 0

  /*-------------------- Jacobian related to DVL --------------------*/

  if(param_msckf_.do_R_I_D){
    // dh()/d(dvl_R_I_D) - (3x3):  - 1/S * R_I_D^T * [R_I_G * v_G_I + [w_I]x * p_I_D]x
    Eigen::Matrix3d d_dvl_r = - temp_1 * toSkewSymmetric(temp_3);
    // id
    auto id_dvl_r = state->getEstimationId(DVL,EST_QUATERNION);
    // stack jacobian matrix
    H.block(0,id_dvl_r,3,3) = d_dvl_r;
  }

  if(param_msckf_.do_p_I_D){
    // dh()/d(dvl_p_I_D) - (3x3):  1/S * R_I_D^T * [w_I]x
    Eigen::Matrix3d d_dvl_p = temp_1 * toSkewSymmetric(w_I);
    // id
    auto id_dvl_p = state->getEstimationId(DVL,EST_POSITION);
    // stack jacobian matrix
    H.block(0,id_dvl_p,3,3) = d_dvl_p;
  }

  // if(param_msckf_.do_time_I_D) {
  //   // dh()/d(dvl_t_I_D) - (3x1):  ??????  
  // }

  if(param_msckf_.do_scale_D) {
    // dh()/d(dvl_scale) - (3x1):  - 1/S^2 * R_I_D^T * (R_I_G * v_G_I + [w_I]x * p_I_D)
    Eigen::Vector3d d_dvl_s = - temp_1 / scale * temp_3;
    // id
    auto id_dvl_s = state->getEstimationId(DVL,EST_SCALE);
    // stack jacobian matrix
    H.block(0,id_dvl_s,3,1) = d_dvl_s;
  }


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

  // std::cout<<"H: "<< H<<std::endl;
  // std::cout<<"K: "<< K<<std::endl;
  // std::cout<<"delta X: "<< delta_X.transpose()<<std::endl;

  /********************************************************************************/
  /*************************** Update state and covariance ************************/
  /********************************************************************************/

  Eigen::Vector3d v_D_hat = temp_1 * temp_3;
  Eigen::Vector3d r = v_D - v_D_hat;
  Eigen::VectorXd delta_X = K * r;

  state->updateState(delta_X);

  // P_k = P_k-1 - K * H * P_k-1

  // state->cov_.triangularView<Eigen::Upper>() -= K * H * state->cov_;
  // state->cov_ = state->cov_.selfadjointView<Eigen::Upper>();

  Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(K.rows(), H.cols()) - K*H;
  state->cov_ = I_KH*state->cov_;

  // Fix the covariance to be symmetric
  Eigen::MatrixXd state_cov_fixed = (state->cov_ + state->cov_.transpose()) / 2.0;
  state->cov_ = state_cov_fixed;

}

void Updater::updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D, bool is_simple) {
  std::ofstream file;

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

  /********************************************************************************/
  /***************************** calculate residual *******************************/
  /********************************************************************************/

  Eigen::Vector3d p_I_D = prior_dvl_.extrinsics.tail(3);
  Eigen::Vector3d temp = toSkewSymmetric(w_I)*p_I_D;

  Eigen::Vector3d v_I_meas;
  // v_I_meas  = v_D;
  v_I_meas << -v_D(0), v_D(1), -v_D(2);

  // Eigen::Vector3d v_I_meas_R = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION))* (v_I_meas * (1417/1500.0) - temp);

  //// v_G_I = R_I_G^T * (S * R_I_D * v_D - [w_I]x * p_I_D)
  v_I_meas = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION)).transpose() * (v_I_meas * (1417/1500.0) - temp);

  Eigen::Vector3d v_I_est = state->getEstimationValue(IMU,EST_VELOCITY);
  count++;

  //// v_m_x,v_m_y_rt,v_m_z_rt,
  //// v_m_x_r,v_m_y_r,v_m_z_r,
  //// v_e_x,v_e_y,v_e_z
  // file.open(file_path, std::ios_base::app);//std::ios_base::app
  // file<<"\n"<<count<<","<<v_I_meas(0)<<","<<v_I_meas(1)<<","<<v_I_meas(2)<<",";
  // file<< v_I_meas_R(0)<<","<<v_I_meas_R(1)<<","<<v_I_meas_R(2)<<",";
  // file<< v_I_est(0)<<","<<v_I_est(1)<<","<<v_I_est(2);
  // file.close();

  Eigen::Vector3d r = v_I_meas - state->getEstimationValue(IMU,EST_VELOCITY);


  /********************************************************************************/
  /************************ update state and covariance ***************************/
  /********************************************************************************/

  // d_x = K * r
  Eigen::VectorXd delta_X = K * r;

  //update state
  state->updateState(delta_X);

  //update covariance

  // P_k = P_k-1 - K * H * P_k-1

  Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(K.rows(), H.cols()) - K*H;
  state->cov_ = I_KH*state->cov_;

  // Fix the covariance to be symmetric
  Eigen::MatrixXd state_cov_fixed = (state->cov_ + state->cov_.transpose()) / 2.0;
  state->cov_ = state_cov_fixed;
}

//! interpolate DVL CP pressure into DVL BT  Velocity, and update together  
void Updater::updatePressure(std::shared_ptr<State> state, const double pres_begin, const double pres_curr, bool is_simple) {
  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  // ================ temp values ================ // 

  // depth measurement in pressure sensor frame
  Eigen::Vector3d p_P;
  p_P << 0, 0, pres_begin - pres_curr;

  // rotation matrix between DVL and pressure
  Eigen::Matrix3d R_D_P;
  R_D_P = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd(prior_dvl_.mount_angle, Eigen::Vector3d::UnitX());

  Eigen::Matrix3d R_I_G_init;
  R_I_G_init <<
          0.999487,0.000000,-0.032023,
          -0.003259,-0.994808,-0.101722,
          -0.031856,0.101774,-0.994297;

  Eigen::Matrix3d R_I_D = toRotationMatrix(prior_dvl_.extrinsics.head(4));

  Eigen::Vector3d p_G_I = state->getEstimationValue(IMU, EST_POSITION).transpose();
  
  // ================ stack H matrix ================ // 
  int size_measurement = 1;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  //! TODO: add access of single state id 
  // auto id_imu_p_z = state->getEstimationId(IMU,EST_POSITION,2);
  auto id_imu_p_z = 5;
  H.block(0,id_imu_p_z,1,1) = Eigen::Matrix<double,1,1>(1.0);

  // std::cout<<"H:\n" << H<<std::endl;
  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/

  // position measurement noise
  Eigen::Matrix<double,1,1> Rn(0.1);

  // K = P * H^T * (H * P * H^T + Rn) ^-1
  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();
  // std::cout<<"K:\n" << K <<std::endl;

  /********************************************************************************/
  /***************************** calculate residual *******************************/
  /********************************************************************************/
  // update function:
  // p_G_I = R_G_I * R_I_D * R_D_P * p_P
  // p_P = R_D_P^T * R_I_D^T * R_G_I^T * p_G_I

  Eigen::Vector3d p_G_I_hat = R_I_G_init.transpose() * R_I_D * R_D_P * p_P;

  Eigen::Matrix<double,1,1> r;
  r << p_G_I_hat(2) - p_G_I(2);

  // Eigen::Vector3d p_G_I_hat = R_I_G_init.transpose() * R_I_D * R_D_P * p_P;
  // std::cout<<"p_P meas in Pressure: "<<p_P.transpose()<<std::endl;
  // std::cout<<"p_G_I_hat esti in global: "<<p_G_I_hat.transpose()<<std::endl;
  // Eigen::Vector3d p_P_hat = R_D_P.transpose() * R_I_D.transpose() * R_I_G_init * p_G_I;
  // std::cout<<"p_G_I in global from IMU: "<<p_G_I.transpose()<<std::endl;
  // std::cout<<"p_P esti in pressure from IMU: "<<p_P_hat.transpose()<<"\n=============\n";

  /********************************************************************************/
  /************************ update state and covariance ***************************/
  /********************************************************************************/

  // d_x = K * r
  Eigen::VectorXd delta_X = K * r;

  //update state
  state->updateState(delta_X);

  //update covariance

  // std::cout<<"cov old: \n"<<state->cov_<<std::endl;

  // P_k = P_k-1 - K * H * P_k-1
  Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(K.rows(), H.cols()) - K*H;
  // std::cout<<"I_KH:\n "<< I_KH<<std::endl;
  state->cov_ = I_KH*state->cov_;

  // Fix the covariance to be symmetric
  Eigen::MatrixXd state_cov_fixed = (state->cov_ + state->cov_.transpose()) / 2.0;
  state->cov_ = state_cov_fixed;

  // std::cout<<"cov new: \n"<<state->cov_<<std::endl;
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