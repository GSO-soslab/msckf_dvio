#include "predictor.h"

namespace msckf_dvio
{

Predictor::Predictor(priorImu prior_imu) : prior_imu_(prior_imu) {}

void Predictor::propagate(std::shared_ptr<State> state, const std::vector<ImuMsg> &data) {


  Eigen::Matrix<double, 15, 15> Phi_summed = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> Qd_summed = Eigen::Matrix<double, 15, 15>::Zero();
  double dt_summed = 0;
  
  //// loop each IMU and propagate the state
  for (size_t i = 0; i < data.size() - 1; i++) {

    // Get the next state Jacobian and noise Jacobian for this IMU reading
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
    Eigen::Matrix<double, 15, 15> Qdi = Eigen::Matrix<double, 15, 15>::Zero();
    propagateState(state, data.at(i), data.at(i + 1), F, Qdi);

    // Next we should propagate our IMU covariance
    // Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
    // Pci' = F*Pci and Pic' = Pic*F.transpose()
    // multiple the each discrete state transition matrices phi to get a total one
    // NOTE: Here we are summing the state transition F so we can do a single mutiplication later
    // NOTE: Phi_summed = Phi_i*Phi_summed
    // NOTE: Q_summed = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
    Phi_summed = F * Phi_summed;
    Qd_summed = F * Qd_summed * F.transpose() + Qdi;
    Qd_summed = 0.5 * (Qd_summed + Qd_summed.transpose());
    dt_summed += data.at(i + 1).time - data.at(i).time;
  }

  // predict covariance (covariance propagation) with our "summed" state transition and IMU noise addition...
  propagateCovariance(state, Phi_summed, Qd_summed);


  // update state time to the IMU's time that we get new sensor measurement
  // most of time, this IMU is interpolated
  state->setTimestamp(data.back().time);

}

void Predictor::propagateCovariance(std::shared_ptr<State> state, 
                                     const Eigen::MatrixXd &Phi,
                                     const Eigen::MatrixXd &Q) {
  // Assert that we have correct sizes
  auto size_imu = state->getImuSize();
  auto size_total = state->cov_.rows();
  auto size_other = size_total - size_imu;

  assert(size_imu == Phi.rows());
  assert(size_imu == Phi.cols());
  assert(size_imu == Q.cols());
  assert(size_imu == Q.rows());

  // Get the location in small phi for each measuring variable
  int id_imu_start = state->getEstimationId(IMU, EST_QUATERNION);
  int id_imu_after = size_imu;

  // Phi_NEW*Covariance*Phi_NEW^t + Q
  Eigen::MatrixXd P_II = Q.selfadjointView<Eigen::Upper>();
  P_II.noalias() += 
    Phi * state->cov_.block(id_imu_start, id_imu_start, size_imu, size_imu) * Phi.transpose();
  // Phi*Covariance_ImuOthers
  Eigen::MatrixXd P_IOthers = Eigen::MatrixXd::Zero(size_imu, size_other);
  P_IOthers.noalias() +=
    Phi * state->cov_.block(id_imu_start, id_imu_after, size_imu, size_other);

  state->cov_.block(id_imu_start, id_imu_start, size_imu,   size_imu)   = P_II;
  state->cov_.block(id_imu_start, id_imu_after, size_imu,   size_other) = P_IOthers;
  state->cov_.block(id_imu_after, id_imu_start, size_other, size_imu)   = P_IOthers.transpose();


}


void Predictor::propagateState(std::shared_ptr<State> state, 
                         const ImuMsg &data_beg, 
                         const ImuMsg &data_end,
                         Eigen::Matrix<double, 15, 15> &F, 
                         Eigen::Matrix<double, 15, 15> &Qd) {
  // Set them to zero
  F.setZero();
  Qd.setZero();

  // Time elapsed over interval
  double dt = data_end.time - data_beg.time;
  // assert(data_end.time>data_beg.time);

  // Corrected imu measurements
  Eigen::Matrix<double, 3, 1> w_hat  = data_beg.w - state->getEstimationValue(IMU, EST_BIAS_G);
  Eigen::Matrix<double, 3, 1> a_hat  = data_beg.a - state->getEstimationValue(IMU, EST_BIAS_A);
  Eigen::Matrix<double, 3, 1> w_hat2 = data_end.w - state->getEstimationValue(IMU, EST_BIAS_G);
  Eigen::Matrix<double, 3, 1> a_hat2 = data_end.a - state->getEstimationValue(IMU, EST_BIAS_A);

  // Compute the new state mean value
  Eigen::Vector4d new_q;
  Eigen::Vector3d new_v, new_p;

  predict_mean_rk4(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);

  // Get the locations of each estimation of the imu state
  int q_id  = state->getEstimationId(IMU, EST_QUATERNION);
  int p_id  = state->getEstimationId(IMU, EST_POSITION);
  int v_id  = state->getEstimationId(IMU, EST_VELOCITY);
  int bg_id = state->getEstimationId(IMU, EST_BIAS_G);
  int ba_id = state->getEstimationId(IMU, EST_BIAS_A);

  // Allocate noise Jacobian
  Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();

  // Now compute Jacobian of new state wrt old state and noise
  Eigen::Matrix<double, 3, 3> R_Gtoi = toRotationMatrix(state->getEstimationValue(IMU, EST_QUATERNION));

  F.block(q_id, q_id, 3, 3) = expSO3(-w_hat * dt);
  F.block(q_id, bg_id, 3, 3).noalias() = -expSO3(-w_hat * dt) * rightJacobSO3(-w_hat * dt) * dt;
  F.block(p_id, q_id, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * toSkewSymmetric(a_hat * dt * dt);
  F.block(p_id, p_id, 3, 3).setIdentity();
  F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
  F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
  F.block(v_id, q_id, 3, 3).noalias() = -R_Gtoi.transpose() * toSkewSymmetric(a_hat * dt);
  F.block(v_id, v_id, 3, 3).setIdentity();
  F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
  F.block(bg_id, bg_id, 3, 3).setIdentity();
  F.block(ba_id, ba_id, 3, 3).setIdentity();

  G.block(q_id, 0, 3, 3) = -expSO3(-w_hat * dt) * rightJacobSO3(-w_hat * dt) * dt;
  G.block(p_id, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
  G.block(v_id, 3, 3, 3) = -R_Gtoi.transpose() * dt;
  G.block(bg_id, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  G.block(ba_id, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();


  // Construct our discrete noise covariance matrix
  // Note that we need to convert our continuous time noises to discrete
  // Equations (129) amd (130) of Trawny tech report

  // N = [n_g, n_a, n_bg, n_ba]T
  Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
  Qc.block(0, 0, 3, 3) = pow(prior_imu_.sigma_w, 2) / dt * Eigen::Matrix<double, 3, 3>::Identity();
  Qc.block(3, 3, 3, 3) = pow(prior_imu_.sigma_a, 2) / dt * Eigen::Matrix<double, 3, 3>::Identity();
  Qc.block(6, 6, 3, 3) = pow(prior_imu_.sigma_wb, 2) * dt * Eigen::Matrix<double, 3, 3>::Identity();
  Qc.block(9, 9, 3, 3) = pow(prior_imu_.sigma_ab, 2) * dt * Eigen::Matrix<double, 3, 3>::Identity();

  // Compute the noise injected into the state over the interval
  Qd = G * Qc * G.transpose();
  Qd = 0.5 * (Qd + Qd.transpose());

  // Now replace imu estimate and fej with propagated values
  Eigen::Matrix<double, 16, 1> imu_predict = state->getImuValue();
  imu_predict.block(0, 0, 4, 1) = new_q;
  imu_predict.block(4, 0, 3, 1) = new_p;
  imu_predict.block(7, 0, 3, 1) = new_v;
  state->setImuValue(imu_predict);
}

void Predictor::predict_mean_rk4(std::shared_ptr<State> state, double dt, 
                                  const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                                  const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, 
                                  Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {

  // Pre-compute things
  Eigen::Vector3d w_hat = w_hat1;
  Eigen::Vector3d a_hat = a_hat1;
  Eigen::Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
  Eigen::Vector3d a_jerk = (a_hat2 - a_hat1) / dt;

  // y0 ================
  Eigen::Vector4d q_0 = state->getEstimationValue(IMU, EST_QUATERNION);
  Eigen::Vector3d p_0 = state->getEstimationValue(IMU, EST_POSITION);
  Eigen::Vector3d v_0 = state->getEstimationValue(IMU, EST_VELOCITY);

  // k1 ================
  Eigen::Vector4d dq_0 = {0, 0, 0, 1};
  Eigen::Vector4d q0_dot = 0.5 * toOmegaMatrix(w_hat) * dq_0;
  Eigen::Vector3d p0_dot = v_0;
  Eigen::Matrix3d R_Gto0 = toRotationMatrix(multiplyQuat(dq_0, q_0));
  Eigen::Vector3d v0_dot = R_Gto0.transpose() * a_hat - prior_imu_.gravity;

  Eigen::Vector4d k1_q = q0_dot * dt;
  Eigen::Vector3d k1_p = p0_dot * dt;
  Eigen::Vector3d k1_v = v0_dot * dt;

  // k2 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Eigen::Vector4d dq_1 = normalizeQuat(dq_0 + 0.5 * k1_q);
  // Eigen::Vector3d p_1 = p_0+0.5*k1_p;
  Eigen::Vector3d v_1 = v_0 + 0.5 * k1_v;

  Eigen::Vector4d q1_dot = 0.5 * toOmegaMatrix(w_hat) * dq_1;
  Eigen::Vector3d p1_dot = v_1;
  Eigen::Matrix3d R_Gto1 = toRotationMatrix(multiplyQuat(dq_1, q_0));
  Eigen::Vector3d v1_dot = R_Gto1.transpose() * a_hat - prior_imu_.gravity;

  Eigen::Vector4d k2_q = q1_dot * dt;
  Eigen::Vector3d k2_p = p1_dot * dt;
  Eigen::Vector3d k2_v = v1_dot * dt;

  // k3 ================
  Eigen::Vector4d dq_2 = normalizeQuat(dq_0 + 0.5 * k2_q);
  // Eigen::Vector3d p_2 = p_0+0.5*k2_p;
  Eigen::Vector3d v_2 = v_0 + 0.5 * k2_v;

  Eigen::Vector4d q2_dot = 0.5 * toOmegaMatrix(w_hat) * dq_2;
  Eigen::Vector3d p2_dot = v_2;
  Eigen::Matrix3d R_Gto2 = toRotationMatrix(multiplyQuat(dq_2, q_0));
  Eigen::Vector3d v2_dot = R_Gto2.transpose() * a_hat - prior_imu_.gravity;

  Eigen::Vector4d k3_q = q2_dot * dt;
  Eigen::Vector3d k3_p = p2_dot * dt;
  Eigen::Vector3d k3_v = v2_dot * dt;

  // k4 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Eigen::Vector4d dq_3 = normalizeQuat(dq_0 + k3_q);
  // Eigen::Vector3d p_3 = p_0+k3_p;
  Eigen::Vector3d v_3 = v_0 + k3_v;

  Eigen::Vector4d q3_dot = 0.5 * toOmegaMatrix(w_hat) * dq_3;
  Eigen::Vector3d p3_dot = v_3;
  Eigen::Matrix3d R_Gto3 = toRotationMatrix(multiplyQuat(dq_3, q_0));
  Eigen::Vector3d v3_dot = R_Gto3.transpose() * a_hat - prior_imu_.gravity;

  Eigen::Vector4d k4_q = q3_dot * dt;
  Eigen::Vector3d k4_p = p3_dot * dt;
  Eigen::Vector3d k4_v = v3_dot * dt;

  // y+dt ================
  Eigen::Vector4d dq = normalizeQuat(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
  new_q = multiplyQuat(dq, q_0);
  new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
  new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
}

void Predictor::augmentDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w) {
  // make sure this clone is new
  auto EST_CLONE_TIME = std::to_string(state->getTimestamp());

  if(state->foundClone(CLONE_DVL, EST_CLONE_TIME)) {
    printf("Predictor error: a clone of substate %d already exist!\n", CLONE_DVL);
    std::exit(EXIT_FAILURE);
  }

  /******************** clone the IMU pose ********************/
  SubState clone;

  auto clone_pose = std::make_shared<PoseJPL>();

  Eigen::Matrix<double, 7, 1> pose;
  pose.block(0, 0, 4, 1) = state->getEstimationValue(IMU, EST_QUATERNION);
  pose.block(4, 0, 3, 1) = state->getEstimationValue(IMU, EST_POSITION);
  clone_pose->setValue(pose);

  int id_curr = state->cov_.rows();
  clone_pose->setId(id_curr);

  state->state_[SubStateName::CLONE_DVL].emplace(EST_CLONE_TIME, clone_pose);


  /******************** augment covariance ********************/
  //  P =|I|* P *|I J^T|
  //     |J|

  // increase covariance matrix size at the end of old matrix, like append
  int size_old = state->cov_.rows();
  int size_clone = clone_pose->getSize();
  state->cov_.conservativeResizeLike(Eigen::MatrixXd::Zero(size_old + size_clone, size_old + size_clone));

  // augment covariance for clone
  auto id_imu = state->getEstimationId(IMU, EST_QUATERNION);
  state->cov_.block(id_curr, id_curr, size_clone, size_clone) = state->cov_.block(0, id_imu, size_clone, size_clone);
  state->cov_.block(0, id_curr, size_old, size_clone) = state->cov_.block(0, id_imu, size_old, size_clone);
  state->cov_.block(id_curr, 0, size_clone, size_old) = state->cov_.block(id_imu, 0, size_clone, size_old);

  // augment covariance for timeoffset calibration
  if(state->params_msckf_.do_time_I_D) {
    auto id_augment = state->getEstimationId(DVL, EST_TIMEOFFSET);

    // Jacobian to augment by
    Eigen::Matrix<double, 6, 1> dnc_dt = Eigen::MatrixXd::Zero(6, 1);
    dnc_dt.block(0, 0, 3, 1) = w;
    dnc_dt.block(3, 0, 3, 1) = state->getEstimationValue(IMU, EST_VELOCITY);
    // Augment covariance with time offset Jacobian
    state->cov_.block(0, id_curr, state->cov_.rows(), 6) +=
      state->cov_.block(0, id_augment, state->cov_.rows(), 1) * dnc_dt.transpose();
    state->cov_.block(id_curr, 0, 6, state->cov_.rows()) +=
      dnc_dt * state->cov_.block(id_augment, 0, 1, state->cov_.rows());
  }

}

} // namespace msckf_dvio