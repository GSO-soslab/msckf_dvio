#include "updater.h"
#include <boost/math/distributions/chi_squared.hpp>

namespace msckf_dvio
{

Updater::Updater(Params &params) : 
  prior_dvl_(params.prior_dvl), 
  prior_pressure_(params.prior_pressure),
  prior_cam_(params.prior_cam), 
  param_msckf_(params.msckf), 
  count(0)
{
  triangulater = std::unique_ptr<FeatureTriangulation>(new FeatureTriangulation(params.triangualtion));

  // Initialize the chi squared test table with confidence level 0.95
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }

}

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
  // DVL BT velocity estimation: z_D = v_D = 1/S * R_I_D^T * (R_I_G * v_G_I + [w_I]x * p_I_D) + n

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

  /*-------------------- Jacobian related to DVL calib --------------------*/

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

  /********************************************************************************/
  /*************************** Update state and covariance ************************/
  /********************************************************************************/

  Eigen::Vector3d v_D_hat = temp_1 * temp_3;
  Eigen::Vector3d r = v_D - v_D_hat;
  Eigen::VectorXd delta_X = K * r;
  // std::cout<<"v_D: "<<v_D.transpose()<<std::endl;
  // std::cout<<"v_D_hat: "<<v_D_hat.transpose()<<std::endl;
  // std::cout<<"delta X: "<< delta_X.transpose()<<std::endl;

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

void Updater::updateDvlSimple(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D, bool is_simple) {

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
  Rn(0,0) = pow(prior_dvl_.sigma_bt(0), 2);
  Rn(1,1) = pow(prior_dvl_.sigma_bt(1), 2);
  Rn(2,2) = pow(prior_dvl_.sigma_bt(2), 2);

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

void Updater::updateDvlPressure(  
  std::shared_ptr<State> state,const Eigen::Vector3d &w_I, 
  const Eigen::Vector3d &v_D,const double pres_begin, const double pres_curr) {
  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  int size_measurement = 4;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  /*-------------------- elements need for DVL velocity estimation --------------------*/
  Eigen::Matrix3d R_I_G = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION));
  Eigen::Vector3d p_G_I = state->getEstimationValue(IMU, EST_POSITION).transpose();
  Eigen::Matrix3d R_I_D = param_msckf_.do_R_I_D ? 
                          toRotationMatrix(state->getEstimationValue(DVL,EST_QUATERNION)) :
                          toRotationMatrix(prior_dvl_.extrinsics.head(4)); 
  Eigen::Vector3d p_I_D = param_msckf_.do_p_I_D ? 
                          state->getEstimationValue(DVL,EST_POSITION) : 
                          prior_dvl_.extrinsics.tail(3);
  double scale = param_msckf_.do_scale_D ? 
                 state->getEstimationValue(DVL,EST_SCALE)(0) : 
                 prior_dvl_.scale;

  // Eigen::Vector3d v_G_I = state->getEstimationValue(IMU,EST_VELOCITY);

  // depth measurement in pressure sensor frame
  Eigen::Vector3d p_P;
  p_P << 0, 0, pres_begin - pres_curr;

  Eigen::Matrix3d R_D_P;
  R_D_P = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd(prior_pressure_.mount_angle, Eigen::Vector3d::UnitX());

  // measurement function:
  // DVL BT velocity estimation: z_D = v_D = 1/S * R_I_D^T * (R_I_G * v_G_I + [w_I]x * p_I_D) + n

  // temp 1: 1/S * R_I_D^T
  Eigen::Matrix3d temp_1 = 1 / scale * R_I_D.transpose();
  // temp 2: R_I_G * v_G_I
  Eigen::Vector3d temp_2 = R_I_G * state->getEstimationValue(IMU,EST_VELOCITY);
  // temp 3: R_I_G * v_G_I + [w_I]x * p_I_D
  Eigen::Vector3d temp_3 = temp_2 + toSkewSymmetric(w_I) * p_I_D;
                          
  int meas_size = 0;
  /*-------------------- position state Jacobian related to IMU --------------------*/
  auto id_imu_p_z = 5;
  H.block(0,id_imu_p_z,1,1) = Eigen::Matrix<double,1,1>(1.0);
  meas_size += 1;

  /*-------------------- Velocity Jacobian related to IMU --------------------*/

  //! TODO: if R_I_D is not calibrated, this update will cased bad state estimation
  //! TODO: try this if well calibrated
  if(param_msckf_.do_R_I_D){
    // State R_I_G(3x3): dh()/d(imu_R_I_G): 1/S * R_I_D^T * [R_I_G * v_G_I]x
    Eigen::Matrix3d d_imu_r = temp_1 * toSkewSymmetric(temp_2);
    // id 
    auto id_imu_r = state->getEstimationId(IMU,EST_QUATERNION);
    // stack jacobian matrix
    H.block(meas_size,id_imu_r,3,3) = d_imu_r;
  }

  // State v_G_I(3x3): dh()/d(imu_v_G_I): 1/S * R_I_D^T * R_I_G
  Eigen::Matrix3d d_imu_v = temp_1 * R_I_G;
  // id
  auto id_imu_v = state->getEstimationId(IMU,EST_VELOCITY);
  // stack jacobian matrix
  H.block(meas_size,id_imu_v,3,3) = d_imu_v;

  meas_size += 3;

  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/
  // K = P * H^T * (H * P * H^T + Rn) ^-1

  // measurement noise(v_x,v_y,v_z,p_z)
  Eigen::Matrix<double, 4, 4> Rn = Eigen::Matrix<double, 4, 4>::Identity();
  Rn(0,0) = pow(prior_pressure_.sigma_pressure, 2);
  Rn(1,1) = pow(prior_dvl_.sigma_bt(0), 2);
  Rn(2,2) = pow(prior_dvl_.sigma_bt(1), 2);
  Rn(3,3) = pow(prior_dvl_.sigma_bt(2), 2);

 // K = P * H^T * (H * P * H^T + Rn) ^-1
  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();

  /********************************************************************************/
  /***************************** calculate residual *******************************/
  /********************************************************************************/

  // [0] Get measurement for position_z

  // p_G_I = R_G_I * R_I_D * R_D_P * p_P
  Eigen::Vector3d p_G_I_meas = R_I_G.transpose() * R_I_D * R_D_P * p_P;

  // [1] Get measurement for velocity
  // v_D as measurement

  // Get total measurement 
  Eigen::Vector4d meas;
  meas << p_G_I_meas(2), v_D(0), v_D(1), v_D(2);

  // Get total estimation
  Eigen::Vector3d v_D_hat = temp_1 * temp_3;

  Eigen::Vector4d est;
  est << p_G_I(2), v_D_hat(0), v_D_hat(1), v_D_hat(2);

  Eigen::Vector4d r = meas - est;

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

void Updater::updateDvlPressureSimple( 
  std::shared_ptr<State> state,const Eigen::Vector3d &w_I, 
  const Eigen::Vector3d &v_D,const double pres_begin, const double pres_curr){
  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  int size_measurement = 4;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  int meas_size = 0;
  // for position_z state
  auto id_imu_p_z = 5;
  H.block(0,id_imu_p_z,1,1) = Eigen::Matrix<double,1,1>(1.0);
  meas_size += 1;

  // for velocity state
  auto id_imu_v = state->getEstimationId(IMU,EST_VELOCITY);
  H.block(meas_size,id_imu_v,3,3) = Eigen::Matrix3d::Identity();
  meas_size += 3;
  
  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/

  // measurement noise(v_x,v_y,v_z,p_z)
  Eigen::Matrix<double, 4, 4> Rn = Eigen::Matrix<double, 4, 4>::Identity();
  Rn(0,0) = pow(prior_pressure_.sigma_pressure, 2);
  Rn(1,1) = pow(prior_dvl_.sigma_bt(0), 2);
  Rn(2,2) = pow(prior_dvl_.sigma_bt(1), 2);
  Rn(3,3) = pow(prior_dvl_.sigma_bt(2), 2);

 // K = P * H^T * (H * P * H^T + Rn) ^-1
  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();

  /********************************************************************************/
  /***************************** calculate residual *******************************/
  /********************************************************************************/

  // Get measurement for position_z
  double p_P_z = pres_begin - pres_curr;

  // Get measurement for velocity

  //// v_G_I = R_I_G^T * (S * R_I_D * v_D - [w_I]x * p_I_D)
  Eigen::Vector3d v_I;
  v_I << -v_D(0), v_D(1), -v_D(2);

  Eigen::Vector3d p_I_D = prior_dvl_.extrinsics.tail(3);
  Eigen::Vector3d temp = toSkewSymmetric(w_I)*p_I_D;

  Eigen::Vector3d v_G_I = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION)).transpose() * ((1417/1500.0) * v_I  - temp);

  // Get total measurement 
  Eigen::Vector4d meas;
  meas << p_P_z, v_G_I(0), v_G_I(1), v_G_I(2);

  // Get total estimation

  Eigen::Vector3d p_G_I_hat = state->getEstimationValue(IMU, EST_POSITION).transpose();
  Eigen::Vector3d v_G_I_hat = state->getEstimationValue(IMU,EST_VELOCITY);

  Eigen::Vector4d est;
  est << p_G_I_hat(2), v_G_I_hat(0), v_G_I_hat(1), v_G_I_hat(2);

  Eigen::Vector4d r = meas - est;

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

void Updater::updatePressureTest(std::shared_ptr<State> state, const double pres_begin, const double pres_curr) {
  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  int size_measurement = 1;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  /*-------------------- elements need for estimation --------------------*/

  Eigen::Matrix3d R_I_G = toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION));

  Eigen::Vector3d p_G_I = state->getEstimationValue(IMU,EST_POSITION);

  // check if do the DVL extrinsics-rotation online calibration
  Eigen::Matrix3d R_I_D = param_msckf_.do_R_I_D ? 
                          toRotationMatrix(state->getEstimationValue(DVL,EST_QUATERNION)) :
                          toRotationMatrix(prior_dvl_.extrinsics.head(4)); 

  // rotation matrix between DVL and pressure
  Eigen::Matrix3d R_D_P;
  R_D_P = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd(prior_pressure_.mount_angle, Eigen::Vector3d::UnitX());

  // third value selection
  Eigen::Matrix<double, 1, 3> s = {0,0,1};

  // measurement function:
  // pressure estimation: z_p = p_P = p_P_in - s * R_D_P^T * R_I_D^T * R_I_G * p_G_I + n

  /*-------------------- Jacobian related to state --------------------*/

  //! TODO: if R_I_D is not calibrated, this update will cased bad state estimation
  //! TODO: try this if well calibrated
  if(param_msckf_.do_R_I_D){
    // State R_I_G(1x3): dh()/d(imu_R_I_G): - s * R_D_P^T * R_I_D^T * [R_I_G * p_G_I]x
    Eigen::Matrix<double, 1, 3> d_imu_r = 
      -s * R_D_P.transpose() * R_I_D.transpose() * toSkewSymmetric(R_I_G*p_G_I);
    // id 
    auto id_imu_r = state->getEstimationId(IMU,EST_QUATERNION);
    // stack jacobian matrix
    H.block(0,id_imu_r,1,3) = d_imu_r;
  }

  // State p_G_I(1x3): dh()/d(imu_p_G_I): -s * R_D_P^T * R_I_D^T * R_I_G
  Eigen::Matrix<double, 1, 3> d_imu_p = 
    -s * R_D_P.transpose() * R_I_D.transpose() * R_I_G;
  // id
  auto id_imu_p = state->getEstimationId(IMU,EST_POSITION);
  // stack jacobian matrix
  H.block(0,id_imu_p,1,3) = d_imu_p;

  // State v_G_I(1x3): dh()/d(imu_v_G_I): 0

  // State bias_w(1x3): dh()/d(imu_bw): 0

  // State bias_a(1x3): dh()/d(imu_ba): 0

  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/
  // K = P * H^T * (H * P * H^T + Rn) ^-1

  // DVL BT measurement noise matrix
  Eigen::Matrix<double,1,1> Rn(prior_pressure_.sigma_pressure * prior_pressure_.sigma_pressure);

  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  // S.triangularView<Eigen::Upper>() = H * state->cov_ * H.transpose();
  // S.triangularView<Eigen::Upper>() += Rn;
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();
  // std::cout<<"H: "<< H<<std::endl;
  // std::cout<<"K: "<< K<<std::endl;

  /********************************************************************************/
  /*************************** Update state and covariance ************************/
  /********************************************************************************/
  Eigen::Vector3d vec_test = R_D_P.transpose() * R_I_D.transpose() * R_I_G * p_G_I;
  double p_P_hat = pres_begin - s * vec_test;
  printf("Pressure est: %f, meas: %f, vec_test:%f,%f,%f\n", p_P_hat, pres_curr, 
        vec_test(0),vec_test(1),vec_test(2));

  Eigen::Matrix<double,1,1> r;
  r << p_P_hat - pres_curr;
  Eigen::VectorXd delta_X = K * r;
  // std::cout<<"v_D: "<<v_D.transpose()<<std::endl;
  // std::cout<<"v_D_hat: "<<v_D_hat.transpose()<<std::endl;
  // std::cout<<"delta X: "<< delta_X.transpose()<<std::endl;

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

void Updater::updatePressureSimple(std::shared_ptr<State> state, const double pres_begin, const double pres_curr) {
  /********************************************************************************/
  /************************* construct Jacobian H matrix **************************/
  /********************************************************************************/

  // depth measurement in pressure sensor frame
  double p_P_z = pres_begin - pres_curr;

  // Useful infomration
  Eigen::Vector3d p_G_I = state->getEstimationValue(IMU, EST_POSITION).transpose();


  // ================ stack H matrix ================ // 
  int size_measurement = 1;
  int size_state = state->getCovCols();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(size_measurement, size_state);

  //! TODO: add access of single state id 
  // auto id_imu_p_z = state->getEstimationId(IMU,EST_POSITION,2);
  auto id_imu_p_z = 5;
  H.block(0,id_imu_p_z,1,1) = Eigen::Matrix<double,1,1>(1.0);

  /********************************************************************************/
  /****************************** Compute Kalman Gain *****************************/
  /********************************************************************************/
  // position measurement noise
  Eigen::Matrix<double,1,1> Rn(prior_pressure_.sigma_pressure * prior_pressure_.sigma_pressure);

  // K = P * H^T * (H * P * H^T + Rn) ^-1
  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  S = H * state->cov_ * H.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();

  /********************************************************************************/
  /***************************** calculate residual *******************************/
  /********************************************************************************/

  Eigen::Matrix<double,1,1> r;
  r << p_P_z - p_G_I(2);

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
          Eigen::AngleAxisd(prior_pressure_.mount_angle, Eigen::Vector3d::UnitX());

  Eigen::Matrix3d R_I_G;
  R_I_G <<toRotationMatrix(state->getEstimationValue(IMU,EST_QUATERNION));
  // R_I_G <<
  //         0.999487,0.000000,-0.032023,
  //         -0.003259,-0.994808,-0.101722,
  //         -0.031856,0.101774,-0.994297;

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
  Eigen::Matrix<double,1,1> Rn(prior_pressure_.sigma_pressure * prior_pressure_.sigma_pressure);
  Eigen::Matrix<double,1,1> Rn1(0.1 * 0.1);
  Eigen::Matrix<double,1,1> Rn2(0.01 * 0.01);

  // K = P * H^T * (H * P * H^T + Rn) ^-1
  Eigen::MatrixXd S(size_measurement, size_measurement);
  S = H * state->cov_ * H.transpose() + Rn;
  Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose();

  //! TEST: replace K(depth) to 0.9 for update, then convert back
  // double k_p_z = K(id_imu_p_z, 0);
  // K(id_imu_p_z, 0) = 0.9;

  //! TEST: replace K(depth) to noise=0.01 for update, then convert back
  // S = H * state->cov_ * H.transpose() + Rn2;
  // Eigen::MatrixXd K_transpose_2 = S.ldlt().solve(H * state->cov_);
  // Eigen::MatrixXd K_2 = K_transpose.transpose();  

  // double k_p_z = K(id_imu_p_z, 0);
  // K(id_imu_p_z, 0) = K_2(id_imu_p_z, 0);

  //! TEST: replace K(depth) to 1 for update, then convert back
  double k_p_z = K(id_imu_p_z, 0);
  K(id_imu_p_z, 0) = 1.0;

  //! TEST: set depth kalman gain to 1
  // K(id_imu_p_z, 0) = 1.0;

  //! TEST: only p_z noise sigma 0.01, others noise sigma 0.1
  // Eigen::MatrixXd S(size_measurement, size_measurement);
  // S = H * state->cov_ * H.transpose() + Rn1;
  // Eigen::MatrixXd K_transpose = S.ldlt().solve(H * state->cov_);
  // Eigen::MatrixXd K = K_transpose.transpose();  

  // S = H * state->cov_ * H.transpose() + Rn2;
  // K_transpose = S.ldlt().solve(H * state->cov_);
  // Eigen::MatrixXd K_temp = K_transpose.transpose();
  // K(id_imu_p_z, 0) = K_temp(id_imu_p_z, 0);

  //! TEST:only p_z, others kalman gain is 0
  // Eigen::MatrixXd K_temp = Eigen::MatrixXd::Zero(size_state,size_measurement);
  // K_temp(id_imu_p_z, 0) = K(id_imu_p_z, 0);
  // K = K_temp;


  // std::cout<<"pz: "<< state->cov_(id_imu_p_z,id_imu_p_z)<<std::endl;
  // std::cout<<"K: "<< K.transpose()<<std::endl;

  /********************************************************************************/
  /***************************** calculate residual *******************************/
  /********************************************************************************/
  // update function:
  // p_G_I = R_G_I * R_I_D * R_D_P * p_P
  // p_P = R_D_P^T * R_I_D^T * R_G_I^T * p_G_I

  Eigen::Vector3d p_G_I_hat = R_I_G.transpose() * R_I_D * R_D_P * p_P;

  Eigen::Matrix<double,1,1> r;
  r << p_G_I_hat(2) - p_G_I(2);
  // std::cout<<"r: "<< r<<std::endl;

  /********************************************************************************/
  /************************ update state and covariance ***************************/
  /********************************************************************************/

  // d_x = K * r
  Eigen::VectorXd delta_X = K * r;
  // std::cout<<"delta_X: "<< delta_X.transpose()<<std::endl;

  state->updateState(delta_X);

  //! TEST: manually set gyro bias z to 0
  // auto imu_value = state->getImuValue();
  // imu_value(12) = 0.0;
  // state->setImuValue(imu_value);

  // update covariance

  // std::cout<<"cov old: \n"<<state->cov_<<std::endl;

  K(id_imu_p_z, 0) = k_p_z;

  // P_k = P_k-1 - K * H * P_k-1
  Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(K.rows(), H.cols()) - K*H;
  // std::cout<<"I_KH:\n "<< I_KH<<std::endl;
  state->cov_ = I_KH*state->cov_;

  // Fix the covariance to be symmetric
  Eigen::MatrixXd state_cov_fixed = (state->cov_ + state->cov_.transpose()) / 2.0;
  state->cov_ = state_cov_fixed;

  // std::cout<<"cov new: \n"<<state->cov_<<std::endl;
}

void Updater::marginalize(std::shared_ptr<State> state, Sensor clone_name, int index) {

  /****************************************************************/
  /******************** marginalize covariance ********************/
  /****************************************************************/

  // find the clone need to marginalize based on given index
  auto marg = std::next(state->state_[clone_name].begin(),index);

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
    state->cov_.block(size_marg_bef + size_marg, 0, size_marg_aft, size_marg_bef);
    //! TODO: don't know why open_vins use this?
    // cov_new.block(0, size_marg_bef, size_marg_bef, size_marg_aft).transpose();

  // P_(x_aft,x_aft)
  cov_new.block(size_marg_bef, size_marg_bef, size_marg_aft, size_marg_aft) = 
    state->cov_.block(size_marg_bef + size_marg, size_marg_bef + size_marg, size_marg_aft, size_marg_aft);

  // set to new covariance
  state->cov_ = cov_new;
  assert(state->cov_.rows() == cov_new.rows());
  assert(state->cov_.cols() == cov_new.cols());

  /****************************************************************/
  /******************* marginalize clone state ********************/
  /****************************************************************/

  // remove clone
  state->state_[clone_name].erase(time_marg);

  // reset id
  for(auto &clones : state->state_[clone_name]) {
    auto id = clones.second->getId();

    if( id > id_marg) 
      clones.second->setId(id - size_marg);
  }
  
}

void Updater::featureTriangulation(    
    std::shared_ptr<State> state, 
    std::vector<Feature> &feat) {

  // -------------------- Get Camera Pose and Clone Time -------------------- //

  // get known information
  double cam_time;
  Eigen::Matrix3d R_I_G, R_C_I;
  Eigen::Vector3d p_G_I, p_C_I;
  R_C_I = param_msckf_.do_R_C_I ? 
          toRotationMatrix(state->getEstimationValue(CAM0,EST_QUATERNION)) :
          toRotationMatrix(prior_cam_.extrinsics.head(4));
  p_C_I = param_msckf_.do_p_C_I ?
          state->getEstimationValue(CAM0,EST_POSITION) :
          prior_cam_.extrinsics.tail(3);

  // get clone pose 
  Eigen::Matrix4d T_G_C = Eigen::Matrix4d::Identity();
  std::unordered_map<double, Eigen::Matrix4d> cam_poses;
  std::vector<double> clone_times;

  for (const auto &clone : state->state_[CLONE_CAM0]) {
    // get clone time and IMU pose
    cam_time = std::stod(clone.first);
    R_I_G = toRotationMatrix(clone.second->getValue().block(0,0,4,1));
    p_G_I = clone.second->getValue().block(4,0,3,1);

    // get camera pose in global frame
    T_G_C.block(0,0,3,3) = R_I_G.transpose() * R_C_I.transpose();
    T_G_C.block(0,3,3,1) = p_G_I - T_G_C.block(0,0,3,3) * p_C_I;

    // insert to container
    cam_poses.insert({cam_time, T_G_C});
  }

  // -------------------- Triangulation -------------------- //

  auto it0 = feat.begin();
  while (it0 != feat.end()) {          

    // [2] Check: if this feature is triangulated 
    if(it0->triangulated) {
      it0++;
      continue;
    }

    // [2] Triangulate the feature
    bool success_tri = true;
    bool success_refine = true;

    success_tri = triangulater->single_triangulation(&*it0, cam_poses);
    success_refine = triangulater->single_gaussnewton(&*it0, cam_poses);

    // [3] Remove the feature if triangulation failed
    if (!success_tri || !success_refine) {
      it0 = feat.erase(it0);
    }
    else {     
      it0++;
    }
  }

  //! TEST: save data
  // file.open(file_path, std::ios_base::app);

  // // loop all the features
  // for(const auto& feat : feat_msckf) {

  //   // loop each measurements
  //   for(size_t i=0; i<feat.timestamps.at(0).size(); i++) {
  //     // update timestamp
  //     file<<std::fixed<<std::setprecision(9);
  //     file<<state->getTimestamp()<<",";
  //     file<<std::fixed<<std::setprecision(6);
  //     // feature id
  //     file<<feat.featid<<",";
  //     // triangulation result
  //     file<<feat.p_FinG(0)<<","<<feat.p_FinG(1)<<","<<feat.p_FinG(2)<<",";

  //     // // measurement timestamp
  //     // file<<std::fixed<<std::setprecision(9);
  //     // file<<feat.timestamps.at(0).at(i)<<std::endl;
  //     // file<<std::fixed<<std::setprecision(4); 

  //     // camera pose
  //     Eigen::Matrix<double, 3, 3> R_G_Ci = cam_poses.at(feat.timestamps.at(0).at(i)).block(0,0,3,3);
  //     Eigen::Matrix<double, 3, 1> p_G_Ci = cam_poses.at(feat.timestamps.at(0).at(i)).block(0,3,3,1);
  //     Eigen::Matrix<double, 4, 1> q_G_toCi = toQuaternion(R_G_Ci.transpose());   

  //     file<< q_G_toCi(0)<<"," << q_G_toCi(1)<<","<< q_G_toCi(2)<<","<< q_G_toCi(3)<<",";
  //     file<< p_G_Ci(0) <<"," << p_G_Ci(1) <<","  << p_G_Ci(2) <<",";

  //     // normalied uv and raw uv
  //     file<<feat.uvs_norm.at(0).at(i)(0)<<","<< feat.uvs_norm.at(0).at(i)(1)<<",";
  //     file<<feat.uvs.at(0).at(i)(0)<<","<< feat.uvs.at(0).at(i)(1)<<"\n";
  //   }
  // }

  // file.close();
}

void Updater::cameraMeasurement(    
    std::shared_ptr<State> state, 
    std::vector<Feature> &features) {

  // -------------------- Triangulation -------------------- //

  // [0] Return if no features
  if (features.empty())
    return;

  // [1] Get Camera Pose and Clone Time

  // get known information
  double cam_time;
  Eigen::Matrix3d R_I_G, R_C_I;
  Eigen::Vector3d p_G_I, p_C_I;
  R_C_I = param_msckf_.do_R_C_I ? 
          toRotationMatrix(state->getEstimationValue(CAM0,EST_QUATERNION)) :
          toRotationMatrix(prior_cam_.extrinsics.head(4));
  p_C_I = param_msckf_.do_p_C_I ?
          state->getEstimationValue(CAM0,EST_POSITION) :
          prior_cam_.extrinsics.tail(3);

  // get clone pose 
  Eigen::Matrix4d T_G_C = Eigen::Matrix4d::Identity();
  std::unordered_map<double, Eigen::Matrix4d> cam_poses;
  std::vector<double> clone_times;

  for (const auto &clone : state->state_[CLONE_CAM0]) {
    // get clone time and IMU pose
    cam_time = std::stod(clone.first);
    R_I_G = toRotationMatrix(clone.second->getValue().block(0,0,4,1));
    p_G_I = clone.second->getValue().block(4,0,3,1);

    // get camera pose in global frame
    T_G_C.block(0,0,3,3) = R_I_G.transpose() * R_C_I.transpose();
    T_G_C.block(0,3,3,1) = p_G_I - T_G_C.block(0,0,3,3) * p_C_I;

    // insert to container
    cam_poses.insert({cam_time, T_G_C});
    clone_times.emplace_back(cam_time);
  }

  // [2] Clean all feature: 
  //      1)make sure they have associated clone; 2)more then 2 for triangulation 
  auto it0 = features.begin();
  while (it0 != features.end()) {

    // clean the feature that don't have the clonetime
    it0->clean_old_measurements(clone_times);

    // count how many measurements
    int num_measurements = 0;
    for (const auto &pair : it0->timestamps) {
      num_measurements += it0->timestamps[pair.first].size();
    }

    // remove if we don't have enough
    //! TODO: add min triangulation feature number to parameters
    if (num_measurements < 2) {
      it0 = features.erase(it0);
    } 
    else {
      it0++;
    }
  }

  // [3] Try to triangulate all MSCKF features

  auto it1 = features.begin();
  while (it1 != features.end()) {

    // triangulate the feature and remove if it fails
    bool success_tri = true;
    bool success_refine = true;

    success_tri = triangulater->single_triangulation(&*it1, cam_poses);

    success_refine = triangulater->single_gaussnewton(&*it1, cam_poses);

    // remove the feature if not a success
    if (!success_tri || !success_refine) {
      it1 = features.erase(it1);
    }
    else {
      it1++;
    }
  }

}

void Updater::updateCam(std::shared_ptr<State> state, std::vector<Feature> &features) {
  if(features.empty()) {
    return;
  }

  // -------------------- Update Equation -------------------- //

  // [0] create the initial H and r matirx 

  // calculate the max possible measurement size
  int max_measurements = 0;
  for(const auto& feat : features) {
    for(const auto& pair : feat.timestamps) {
      max_measurements += 2 * pair.second.size();
    }
  }
  assert(max_measurements > 2);

  // calculate max possible state size, covariance size
  int max_state = state->getCovCols();

  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_measurements, max_state);
  Eigen::VectorXd residual = Eigen::VectorXd::Zero(max_measurements);
  int stack_count = 0;

  // [1] stack all the each feature jacobian 
  auto it2 = features.begin();
  while (it2 != features.end()) {
  // for(const auto& feat : features) {
    Eigen::MatrixXd H_xj;
    Eigen::MatrixXd H_fj;
    Eigen::VectorXd r_j;

    // stack on all the measurements for single feature
    featureJacobian(state, *it2, H_xj, H_fj, r_j);

    // nullspace projection to remove global feature related
    nullspace_project(H_fj, H_xj, r_j);
    // nullspace_project_inplace(H_fj, H_xj, r_j);

    // Mahalanobis gating test
    // Eq.(16) Mingyang Li et al. ConsistentVIO_2013_IJRR
    Eigen::MatrixXd S = H_xj * state->cov_ * H_xj.transpose();
    S.diagonal() += prior_cam_.noise * prior_cam_.noise * Eigen::VectorXd::Ones(S.rows());
    double gamma = r_j.dot(S.llt().solve(r_j));

    // 95% chi^2 table threshold
    double chi2_check;
    if (r_j.rows() < 500) {
      chi2_check = chi_squared_table[r_j.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(r_j.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      printf("Warning: chi2_check over the residual limit - %d\n", (int)r_j.rows());
    }

    // Check if we should delete or not
    if (gamma > chi2_check) {
      printf("chi2 failed: id:%ld \n", (*it2).featid);
      it2 = features.erase(it2);
      continue;
    }

    // stack one feature's measurements
    H_x.block(stack_count, 0, H_xj.rows(), H_xj.cols()) = H_xj;
    residual.block(stack_count, 0, r_j.rows(), 1) = r_j;
    stack_count += r_j.rows();

    it2++;
  }

  // [2] resize the matirx to the actual(dimension reduced by nullspace project, gating test filter)
  if (stack_count < 1) {
    return;
  }

  assert(stack_count <= max_measurements);
  H_x.conservativeResize(stack_count, max_state);
  residual.conservativeResize(stack_count, 1);
  if(stack_count > 1500) {
    printf("warning: large size of measurement jacobian = %d", stack_count);
  }

  // [3] compress
  compress(H_x, residual);
  // compress_inplace(H_x, residual);  

  // -------------------- Update EKF:Compute Kalman Gain -------------------- //

  // K = P * H^T * (H * P * H^T + Rn) ^-1

  Eigen::MatrixXd Rn = prior_cam_.noise * prior_cam_.noise * 
    Eigen::MatrixXd::Identity(residual.rows(), residual.rows());

  Eigen::MatrixXd S(Rn.rows(), Rn.rows());
  S = H_x * state->cov_ * H_x.transpose() + Rn;

  Eigen::MatrixXd K_transpose = S.ldlt().solve(H_x * state->cov_);
  Eigen::MatrixXd K = K_transpose.transpose(); 


  // -------------------- Update EKF: state and covariance -------------------- //

  // d_x = K * r
  Eigen::VectorXd delta_X = K * residual;

  //update state
  state->updateState(delta_X);

  //update covariance

  // P_k = P_k-1 - K * H * P_k-1

  Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(K.rows(), H_x.cols()) - K*H_x;
  state->cov_ = I_KH*state->cov_;

  // Fix the covariance to be symmetric
  Eigen::MatrixXd state_cov_fixed = (state->cov_ + state->cov_.transpose()) / 2.0;
  state->cov_ = state_cov_fixed;

  // check
  assert(!state->foundSPD("cam_update"));
}

void Updater::updateCamPart(
    std::shared_ptr<State> state, 
    std::vector<Feature> &features) {

  if(features.empty()) {
    return;
  }

  // -------------------- Update Equation -------------------- //

  // [0] create the initial H and r matirx 

  // calculate the max possible measurement size
  int max_measurements = 0;
  for(const auto& feat : features) {
    for(const auto& pair : feat.timestamps) {
      max_measurements += 2 * pair.second.size();
    }
  }
  assert(max_measurements > 2);

  // calculate max possible state size, covariance size
  int max_state = state->getCovCols();

  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_measurements, max_state);
  Eigen::VectorXd residual = Eigen::VectorXd::Zero(max_measurements);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  int ct_jacob = 0;
  int ct_meas = 0;

  // [1] stack all the each feature jacobian 
  auto it2 = features.begin();
  while (it2 != features.end()) {
  // for(const auto& feat : features) {
    Eigen::MatrixXd H_xj;
    Eigen::MatrixXd H_fj;
    Eigen::VectorXd r_j;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // stack on all the measurements for single feature
    // featureJacobian(state, *it2, H_xj, H_fj, r_j);
    featureJacobianPart(state, *it2, H_xj, H_fj, r_j, Hx_order);
    // std::cout<<"H_jx: \n"<< H_xj <<std::endl;

    // nullspace projection to remove global feature related
    // nullspace_project(H_fj, H_xj, r_j);
    nullspace_project_inplace(H_fj, H_xj, r_j);

    // Mahalanobis gating test to filter bad measurement
    if(!chiSquareTest(state, H_xj, r_j, Hx_order)) {
      printf("chi2 failed: id:%ld \n", (*it2).featid);
      it2 = features.erase(it2);
      continue;
    }

    // stack one feature's measurements
    // This big H matrix only has state involved in the Jacobian derivative
    // This order is not same as the order of state vector
    size_t ct_hx = 0;
    for (const auto &var : Hx_order) {

      // Ensure that this variable is in our Jacobian
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->getSize();
      }

      // Append to our large Jacobian
      H_x.block(ct_meas, Hx_mapping[var], H_xj.rows(), var->getSize()) = 
        H_xj.block(0, ct_hx, H_xj.rows(), var->getSize());
      ct_hx += var->getSize();
    }    

    residual.block(ct_meas, 0, r_j.rows(), 1) = r_j;
    ct_meas += r_j.rows();

    it2++;
  }

  // [2] resize the matirx to the actual(dimension reduced by nullspace project, gating test filter)
  if (ct_meas < 1) {
    printf("ct_meas<1, return\n");
    return;
  }

  assert(ct_meas <= max_measurements);
  assert(ct_jacob <= max_state);
  H_x.conservativeResize(ct_meas, ct_jacob);
  residual.conservativeResize(ct_meas, 1);
  if(ct_meas > 1500) {
    printf("warning: large size of measurement jacobian = %d", ct_meas);
  }

  // [3] compress
  // compress(H_x, residual);
  compress_inplace(H_x, residual);
  if (H_x.rows() < 1) {
    printf("cam update: H_x.rows()<1, return\n");
    return;
  }

  // std::cout<<"residual: \n"<<residual.transpose()<<std::endl;

 // [4] update
  Eigen::MatrixXd Rn = prior_cam_.noise * prior_cam_.noise * 
    Eigen::MatrixXd::Identity(residual.rows(), residual.rows());

  update(state, Hx_order_big, H_x, residual, Rn);
}

void Updater::featureJacobianPart(
  std::shared_ptr<State> state, const Feature &feature, 
  Eigen::MatrixXd &H_x, Eigen::MatrixXd &H_f, 
  Eigen::VectorXd & res, std::vector<std::shared_ptr<Type>> &x_order) {

  /***************************************************************************/
  /*                      Select the state variable involved                 */
  /***************************************************************************/

  // total number of measurements for single feature
  int size_measurement = 0;
  for (auto const &pair : feature.timestamps) {
    size_measurement += 2 * (int)pair.second.size();
  }

  // select the states involved with this feature
  int total_hx = 0;
  std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
  for(auto const &pair : feature.timestamps) {

    //! TODO: set calibration extrinsics as PoseJPL

    for (size_t m = 0; m < feature.timestamps.at(pair.first).size(); m++) {
      // Add this clone if it is not added already
      auto EST_CLONE_TIME = toCloneStamp(feature.timestamps.at(pair.first).at(m));
      auto clone_Ii = state->getEstimation(CLONE_CAM0,EST_CLONE_TIME);

      if (map_hx.find(clone_Ii) == map_hx.end()) {
        map_hx.insert({clone_Ii, total_hx});
        x_order.push_back(clone_Ii);
        total_hx += clone_Ii->getSize();
      }
    }
  }

  /***************************************************************************/
  /*                     Allocate our residual and Jacobians                 */
  /***************************************************************************/

  // setup matrix size
  H_x = Eigen::MatrixXd::Zero(size_measurement, total_hx);
  H_f = Eigen::MatrixXd::Zero(size_measurement, 3);
  res = Eigen::VectorXd::Zero(size_measurement);
  int count = 0;

  for(auto const &pair : feature.timestamps) {
    // get information
    Eigen::Matrix3d R_C_I = param_msckf_.do_R_C_I ?
      toRotationMatrix(state->getEstimationValue(CAM0,EST_QUATERNION)) :
      toRotationMatrix(prior_cam_.extrinsics.head(4)); 

    Eigen::Vector3d p_C_I = param_msckf_.do_p_C_I ? 
      state->getEstimationValue(CAM0,EST_POSITION) : 
      prior_cam_.extrinsics.tail(3);

    Eigen::Matrix3d R_Ik_G;
    Eigen::Vector3d p_G_Ik;

    //
    for (size_t m = 0; m < feature.timestamps.at(pair.first).size(); m++) {
      // get clone IMU pose at clone time
      auto EST_CLONE_TIME = toCloneStamp(feature.timestamps.at(pair.first).at(m));
      auto clone_Ii = state->getEstimation(CLONE_CAM0,EST_CLONE_TIME);

      R_Ik_G = toRotationMatrix(clone_Ii->getValue().block(0,0,4,1));
      p_G_Ik = clone_Ii->getValue().block(4,0,3,1);

      // get feature on current camera frame
      Eigen::Vector3d p_C_F = R_C_I * R_Ik_G * (feature.p_FinG - p_G_Ik) + p_C_I;

      /*-------------------- Jacobian related to feature in camera frame --------------------*/

      Eigen::Matrix<double, 2, 3> dhp_dp_C_F = Eigen::Matrix<double, 2, 3>::Zero();
      dhp_dp_C_F(0, 0) = 1 / p_C_F(2);
      dhp_dp_C_F(1, 1) = 1 / p_C_F(2);
      dhp_dp_C_F(0, 2) = - p_C_F(0) / (p_C_F(2) * p_C_F(2));
      dhp_dp_C_F(1, 2) = - p_C_F(1) / (p_C_F(2) * p_C_F(2));

      /*-------------------- Jacobian related to IMU clone --------------------*/
      // dh()/d(R_Ik_G)
      Eigen::Matrix3d dht_dR_Ik_G = R_C_I * toSkewSymmetric(R_Ik_G * (feature.p_FinG - p_G_Ik));
      // dh()/d(p_G_Ik)
      Eigen::Matrix3d dht_dp_G_Ik = - R_C_I * R_Ik_G;
      // stack one for clone pose
      Eigen::Matrix<double, 2, 6> dht_dclone = Eigen::Matrix<double, 2, 6>::Zero();
      dht_dclone.block(0, 0, 2, 3) = dhp_dp_C_F * dht_dR_Ik_G;
      dht_dclone.block(0, 3, 2, 3) = dhp_dp_C_F * dht_dp_G_Ik;
      // stack jacobian matrix
      H_x.block(2 * count, map_hx[clone_Ii], 2, clone_Ii->getSize()).noalias() = dht_dclone;

      /*-------------------- Jacobian related to features in global frame --------------------*/
      // dh()/d(p_G_F)
      Eigen::Matrix3d dht_dp_G_F = R_C_I * R_Ik_G;
      // stack jacobian
      H_f.block(2 * count, 0, 2, H_f.cols()).noalias() = dhp_dp_C_F * dht_dp_G_F;

      /*-------------------- residual --------------------*/
      // residual = measurement - estimation
      res.block(2 * count, 0, 2, 1).noalias() = 
          Eigen::Vector2d(feature.uvs_norm.at(pair.first).at(m)(0), 
                          feature.uvs_norm.at(pair.first).at(m)(1)) - 
          Eigen::Vector2d(p_C_F(0)/p_C_F(2), p_C_F(1)/p_C_F(2));

      // add for each measurement
      count++;
    }
  }
}

void Updater::featureJacobian(std::shared_ptr<State> state, const Feature &feature, 
                              Eigen::MatrixXd &H_x, Eigen::MatrixXd &H_f, 
                              Eigen::VectorXd & res) {

  // total number of measurements for single feature
  int size_measurement = 0;
  for (auto const &pair : feature.timestamps) {
    size_measurement += 2 * (int)pair.second.size();
  }

  // calculate max possible state size, covariance size
  int size_state = state->getCovCols();

  // setup matrix size
  H_x = Eigen::MatrixXd::Zero(size_measurement, size_state);
  H_f = Eigen::MatrixXd::Zero(size_measurement, 3);
  res = Eigen::VectorXd::Zero(size_measurement);
  int count = 0;

  for(auto const &pair : feature.timestamps) {

    // get information
    Eigen::Matrix3d R_C_I = param_msckf_.do_R_C_I ?
      toRotationMatrix(state->getEstimationValue(CAM0,EST_QUATERNION)) :
      toRotationMatrix(prior_cam_.extrinsics.head(4)); 

    Eigen::Vector3d p_C_I = param_msckf_.do_p_C_I ? 
      state->getEstimationValue(CAM0,EST_POSITION) : 
      prior_cam_.extrinsics.tail(3);

    Eigen::Matrix3d R_Ik_G;
    Eigen::Vector3d p_G_Ik;

    for (size_t m = 0; m < feature.timestamps.at(pair.first).size(); m++) {
      
      // get clone IMU pose at clone time
      auto EST_CLONE_TIME = toCloneStamp(feature.timestamps.at(pair.first).at(m));
      auto clone_pose = state->getEstimationValue(CLONE_CAM0,EST_CLONE_TIME);
      R_Ik_G = toRotationMatrix(clone_pose.block(0,0,4,1));
      p_G_Ik = clone_pose.block(4,0,3,1);

      // get feature on current camera frame
      Eigen::Vector3d p_C_F = R_C_I * R_Ik_G * (feature.p_FinG - p_G_Ik) + p_C_I;

      // H = [H_imu H_calib H_clone H_f]

      /*-------------------- Jacobian related to feature in camera frame --------------------*/

      Eigen::Matrix<double, 2, 3> dhp_dp_C_F = Eigen::Matrix<double, 2, 3>::Zero();
      dhp_dp_C_F(0, 0) = 1 / p_C_F(2);
      dhp_dp_C_F(1, 1) = 1 / p_C_F(2);
      dhp_dp_C_F(0, 2) = - p_C_F(0) / (p_C_F(2) * p_C_F(2));
      dhp_dp_C_F(1, 2) = - p_C_F(1) / (p_C_F(2) * p_C_F(2));

      /*-------------------- Jacobian related to Camera calibration --------------------*/
      if(param_msckf_.do_R_C_I) {
        // dh()/d(R_C_I) 
        Eigen::Matrix3d dht_dR_C_I = toSkewSymmetric(R_C_I * R_Ik_G * (feature.p_FinG - p_G_Ik));
        // id
        auto id_R_C_I = state->getEstimationId(CAM0,EST_QUATERNION);
        // stack jacobian matrix
        H_x.block(2 * count, id_R_C_I, 2, 3) = dhp_dp_C_F * dht_dR_C_I;
      }

      if(param_msckf_.do_p_C_I) {
        // dh()/d(p_C_I)  
        Eigen::Matrix3d dht_dp_C_I = Eigen::Matrix3d::Identity();
        // id
        auto id_p_C_I = state->getEstimationId(CAM0,EST_POSITION);
        // stack jacobian matrix
        H_x.block(2 * count, id_p_C_I, 2, 3) = dhp_dp_C_F * dht_dp_C_I;
      }

      /*-------------------- Jacobian related to IMU clone --------------------*/
      // dh()/d(R_Ik_G)
      Eigen::Matrix3d dht_dR_Ik_G = R_C_I * toSkewSymmetric(R_Ik_G * (feature.p_FinG - p_G_Ik));
      // dh()/d(p_G_Ik)
      Eigen::Matrix3d dht_dp_G_Ik = - R_C_I * R_Ik_G;
      // stack one for clone pose
      Eigen::Matrix<double, 2, 6> dht_dclone = Eigen::Matrix<double, 2, 6>::Zero();
      dht_dclone.block(0, 0, 2, 3) = dhp_dp_C_F * dht_dR_Ik_G;
      dht_dclone.block(0, 3, 2, 3) = dhp_dp_C_F * dht_dp_G_Ik;
      // id
      auto id_clone = state->getEstimationId(CLONE_CAM0,EST_CLONE_TIME);
      // stack jacobian matrix
      H_x.block(2 * count, id_clone, 2, 6) = dht_dclone;

      /*-------------------- Jacobian related to features in global frame --------------------*/
      // dh()/d(p_G_F)
      Eigen::Matrix3d dht_dp_G_F = R_C_I * R_Ik_G;
      // stack jacobian
      H_f.block(2 * count, 0, 2, 3) = dhp_dp_C_F * dht_dp_G_F;

      /*-------------------- residual --------------------*/
      // residual = measurement - estimation
      res.block(2 * count, 0, 2, 1) = 
          Eigen::Vector2d(feature.uvs_norm.at(pair.first).at(m)(0), 
                          feature.uvs_norm.at(pair.first).at(m)(1)) - 
          Eigen::Vector2d(p_C_F(0)/p_C_F(2), p_C_F(1)/p_C_F(2));

      // add for each measurement
      count++;
    }
  }
}

// from msckf_vio
void Updater::nullspace_project(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Project the residual and Jacobians onto the nullspace of H_fj.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(H_f, Eigen::ComputeFullU | Eigen::ComputeThinV);
  Eigen::MatrixXd A = svd_helper.matrixU().rightCols(res.size() - 3);

  H_x = A.transpose() * H_x;
  res = A.transpose() * res;
}

// from open_vins
void Updater::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Apply the left nullspace of H_f to all variables
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_f.cols(); ++n) {
    for (int m = (int)H_f.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
  // NOTE: need to eigen3 eval here since this experiences aliasing!
  // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

  // Sanity check
  assert(H_x.rows() == res.rows());
}

bool Updater::chiSquareTest(
  std::shared_ptr<State> state, const Eigen::MatrixXd &H_x, 
  const Eigen::VectorXd &r, std::vector<std::shared_ptr<Type>> x_order) {

  // Mahalanobis gating test
  // Eq.(16) Mingyang Li et al. ConsistentVIO_2013_IJRR

  // // use entire H_x matrix including the IMU state
  // Eigen::MatrixXd S = H_x * state->getCov() * H_x.transpose();
  // S.diagonal() += prior_cam_.noise * prior_cam_.noise * Eigen::VectorXd::Ones(S.rows());
  // double gamma = r.dot(S.llt().solve(r));

  // use only involved states
  Eigen::MatrixXd P_involved = state->getPartCov(x_order);
  Eigen::MatrixXd S = H_x * P_involved * H_x.transpose();
  S.diagonal() += prior_cam_.noise * prior_cam_.noise * Eigen::VectorXd::Ones(S.rows());
  double gamma = r.dot(S.llt().solve(r));

  // std::cout<<"P_involved: \n"<< P_involved<<std::endl;
  // std::cout<<"H_x: \n" << H_x <<std::endl;
  // std::cout<<"r: \n" << r <<std::endl;

  // std::exit(EXIT_FAILURE);

  // 95% chi^2 table threshold
  double chi2_check;
  if (r.rows() < 500) {
    chi2_check = chi_squared_table[r.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(r.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    printf("Warning: chi2_check over the residual limit - %d\n", (int)r.rows());
  }

  // Check if we should delete or not
  if (gamma > chi2_check) {
    return false;
  }
  else {
    return true;
  }
}

// from: msckf_vio
void Updater::compress(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {
  // Eq.(27) Anastasios Mourikis et al. MSCKF_ICRA_2007
  if (H_x.rows() <= H_x.cols())
    return;

  // Convert H to a sparse matrix.
  Eigen::SparseMatrix<double> H_sparse = H_x.sparseView();

  // Perform QR decompostion on H_sparse.
  Eigen::SPQR<Eigen::SparseMatrix<double> > spqr_helper;
  spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
  spqr_helper.compute(H_sparse);

  Eigen::MatrixXd H_temp;
  Eigen::VectorXd r_temp;
  (spqr_helper.matrixQ().transpose() * H_x).evalTo(H_temp);
  (spqr_helper.matrixQ().transpose() * res).evalTo(r_temp);

  H_x = H_temp.topRows(H_x.cols());
  res = r_temp.head(H_x.cols());

  // Eigen::HouseholderQR<Eigen::MatrixXd> qr_helper(H_x);
  // Eigen::MatrixXd Q = qr_helper.householderQ();
  // Eigen::MatrixXd Q1 = Q.leftCols(H_x.cols());

  // H_x = Q1.transpose() * H_x;
  // res = Q1.transpose() * res;
}

// from: open_vins
void Updater::compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Return if H_x is a fat matrix (there is no need to compress in this case)
  if (H_x.rows() <= H_x.cols())
    return;

  // Do measurement compression through givens rotations
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_x.cols(); n++) {
    for (int m = (int)H_x.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_x(m - 1, n), H_x(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_x.block(m - 1, n, 2, H_x.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // If H is a fat matrix, then use the rows
  // Else it should be same size as our state
  int r = std::min(H_x.rows(), H_x.cols());

  // Construct the smaller jacobian and residual after measurement compression
  assert(r <= H_x.rows());
  H_x.conservativeResize(r, H_x.cols());
  res.conservativeResize(r, res.cols());
}

void Updater::update(
    std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R) {

  assert(res.rows() == R.rows());
  assert(H.rows() == res.rows());

  /***************************************************************************/
  /*                         Kalman Gain Calculation                         */
  /***************************************************************************/
  // K = (P * H^T) * (H * P * H^T + Rn) ^{-1}
  //        |                |
  // K =    M      *         S^{-1}


  /* ---------- Get the M part ---------- */

  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->getCovRows(), res.rows());

  // Get the location in small jacobian for each measuring variable
  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->getSize();
  }

  // Get estimation variable from the state vector
  std::vector<std::shared_ptr<Type>> full_states;
  for(const auto& [sensor_name, sensor_state] : state->state_) {
    for(const auto& [est_name, est_state] : sensor_state) {
      full_states.push_back(est_state);
    }
  }

  // For each active variable find its M = P*H^T
  for (const auto& var : full_states) {
    // Sum up effect of each subjacobian = K_i= \sum_m (P_im Hm^T)
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->getSize(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      std::shared_ptr<Type> meas_var = H_order[i];
      M_i.noalias() += state->cov_.block(var->getId(), meas_var->getId(), var->getSize(), meas_var->getSize()) *
                       H.block(0, H_id[i], H.rows(), meas_var->getSize()).transpose();
    }
    M_a.block(var->getId(), 0, var->getSize(), res.rows()) = M_i;
  }

  /* ---------- Get the S part ---------- */

  // Get covariance of the involved terms
  Eigen::MatrixXd P_small = state->getPartCov(H_order);

  Eigen::MatrixXd S(R.rows(), R.rows());
  S.triangularView<Eigen::Upper>() = H * P_small * H.transpose();
  S.triangularView<Eigen::Upper>() += R;

  // Invert our S 
  Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
  S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);

  /* ---------- Get the Kalman Gain ---------- */
  Eigen::MatrixXd K = M_a * Sinv.selfadjointView<Eigen::Upper>();

  /***************************************************************************/
  /*                               EKF Update                                */
  /***************************************************************************/

  // update state
  // d_x = K * r
  Eigen::VectorXd delta_X = K * res;

  // constrain camera z not to update
  // delta_X(5) = 0.0;

  state->updateState(delta_X);

  // update covariance
  // P_k = P_k-1 - K * H * P_k-1
  state->cov_.triangularView<Eigen::Upper>() -= K * M_a.transpose();
  state->cov_ = state->cov_.selfadjointView<Eigen::Upper>();

  assert(!state->foundSPD("cam_update"));
}

} // namespace msckf_dvio