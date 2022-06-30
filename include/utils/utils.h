#ifndef MSCKF_UTILS_H_
#define MSCKF_UTILS_H_

#include <Eigen/Eigen>
#include "types/msgs.h"

namespace msckf_dvio {

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


// Returns skew-symmetric form of a 3-d vector
inline Eigen::Matrix3d toSkewSymmetric(const Eigen::Vector3d& vec) {

  Eigen::Matrix3d mat;
  mat <<      0, -vec(2),  vec(1),
         vec(2),       0, -vec(0),
        -vec(1),  vec(0),       0;

  return mat;
}

/**
 * @brief Integrated quaternion from angular velocity, Compute the omega-matrix of a 3-d vector omega
 *
 * See equation (48) of trawny tech report [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
 *
 */
inline Eigen::Matrix4d toOmegaMatrix(const Eigen::Vector3d & omega) {
  Eigen::Matrix4d bigOmega;
  bigOmega.setZero();

  bigOmega.block<3, 3>(0, 0) = -toSkewSymmetric(omega);
  bigOmega.block<3, 1>(0, 3) = omega;
  bigOmega.block<1, 3>(3, 0) = -omega.transpose();

  return bigOmega;
}

/**
 * @brief Returns a JPL quaternion from a rotation matrix
 *
 * This is based on the equation 74 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
 * In the implementation, we have 4 statements so that we avoid a division by zero and
 * instead always divide by the largest diagonal element. This all comes from the
 * definition of a rotation matrix, using the diagonal elements and an off-diagonal.
 * \f{align*}{
 *  \mathbf{R}(\bar{q})=
 *  \begin{bmatrix}
 *  q_1^2-q_2^2-q_3^2+q_4^2 & 2(q_1q_2+q_3q_4) & 2(q_1q_3-q_2q_4) \\
 *  2(q_1q_2-q_3q_4) & -q_2^2+q_2^2-q_3^2+q_4^2 & 2(q_2q_3+q_1q_4) \\
 *  2(q_1q_3+q_2q_4) & 2(q_2q_3-q_1q_4) & -q_1^2-q_2^2+q_3^2+q_4^2
 *  \end{bmatrix}
 * \f}
 *
 * @param[in] rot 3x3 rotation matrix
 * @return 4x1 quaternion
 */
inline Eigen::Matrix<double, 4, 1> toQuaternion(const Eigen::Matrix<double, 3, 3> &rot) {
  Eigen::Matrix<double, 4, 1> q;
  double T = rot.trace();
  if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2))) {
    // cout << "case 1- " << endl;
    q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
    q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
    q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
    q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

  } else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2))) {
    // cout << "case 2- " << endl;
    q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
    q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
    q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
  } else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1))) {
    // cout << "case 3- " << endl;
    q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
    q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
    q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
  } else {
    // cout << "case 4- " << endl;
    q(3) = sqrt((1 + T) / 4);
    q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
    q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
    q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
  }
  if (q(3) < 0) {
    q = -q;
  }
  // normalize and return
  q = q / (q.norm());
  return q;
}

/**
 * @brief Converts JPL quaterion to SO(3) rotation matrix
 *
 * This is based on equation 62 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf):
 *
 * @param[in] q JPL quaternion
 * @return 3x3 SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> toRotationMatrix(const Eigen::Matrix<double, 4, 1> &q) {

  Eigen::Matrix<double, 3, 3> q_x = toSkewSymmetric(q.block(0, 0, 3, 1));
  Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                        2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
  return Rot;
}

/**
 * @brief Multiply two JPL quaternions
 *
 * This is based on equation 9 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
 * We also enforce that the quaternion is unique by having q_4 be greater than zero.
 *
 * @param[in] q First JPL quaternion
 * @param[in] p Second JPL quaternion
 * @return 4x1 resulting p*q quaternion
 */
inline Eigen::Matrix<double, 4, 1> multiplyQuat(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p) {
  Eigen::Matrix<double, 4, 1> q_t;
  Eigen::Matrix<double, 4, 4> Qm;
  // create big L matrix
  Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - toSkewSymmetric(q.block(0, 0, 3, 1));
  Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
  Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
  Qm(3, 3) = q(3, 0);
  q_t = Qm * p;
  // ensure unique by forcing q_4 to be >0
  if (q_t(3, 0) < 0) {
    q_t *= -1;
  }
  // normalize and return
  return q_t / q_t.norm();
}

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
inline Eigen::Matrix<double, 4, 1> normalizeQuat(Eigen::Matrix<double, 4, 1> q_t) {
  if (q_t(3, 0) < 0) {
    q_t *= -1;
  }
  return q_t / q_t.norm();
}


/**
 * @brief SO(3) matrix exponential
 *
 * SO(3) matrix exponential mapping from the vector to SO(3) lie group.
 * This formula ends up being the [Rodrigues formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula).
 * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 15.
 * http://ethaneade.com/lie.pdf
 *
 * @param[in] w 3x1 vector we will take the exponential of
 * @return SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> expSO3(const Eigen::Matrix<double, 3, 1> &w) {
  // get theta
  Eigen::Matrix<double, 3, 3> w_x = toSkewSymmetric(w);
  double theta = w.norm();
  // Handle small angle values
  double A, B;
  if (theta < 1e-12) {
    A = 1;
    B = 0.5;
  } else {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
  }
  // compute so(3) rotation
  Eigen::Matrix<double, 3, 3> R;
  if (theta == 0) {
    R = Eigen::MatrixXd::Identity(3, 3);
  } else {
    R = Eigen::MatrixXd::Identity(3, 3) + A * w_x + B * w_x * w_x;
  }
  return R;
}

/**
 * @brief Computes left Jacobian of SO(3)
 *
 * The left Jacobian of SO(3) is defined equation (7.77b) in [State Estimation for
 * Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) by Timothy D. Barfoot.
 *
 * @param w axis-angle
 * @return The left Jacobian of SO(3)
 */
inline Eigen::Matrix<double, 3, 3> leftJacobSO3(Eigen::Matrix<double, 3, 1> w) {
  double theta = w.norm();
  if (theta < 1e-12) {
    return Eigen::MatrixXd::Identity(3, 3);
  } else {
    Eigen::Matrix<double, 3, 1> a = w / theta;
    Eigen::Matrix<double, 3, 3> J = sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) + (1 - sin(theta) / theta) * a * a.transpose() +
                                    ((1 - cos(theta)) / theta) * toSkewSymmetric(a);
    return J;
  }
}

/**
 * @brief Computes right Jacobian of SO(3)
 *
 * The right Jacobian of SO(3) is related to the left by Jl(-w)=Jr(w).
 * See equation (7.87) in [State Estimation for Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) by Timothy D. Barfoot.
 * See @ref Jl_so3() for the definition of the left Jacobian of SO(3).
 *
 * @param w axis-angle
 * @return The right Jacobian of SO(3)
 */
inline Eigen::Matrix<double, 3, 3> rightJacobSO3(Eigen::Matrix<double, 3, 1> w) { return leftJacobSO3(-w); }
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/**
 * @brief Nice helper function that will linearly interpolate between two imu messages.
 *
 * This should be used instead of just "cutting" imu messages that bound the camera times
 * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
 *
 * @param imu_beg imu at begining of interpolation interval
 * @param imu_end imu at end of interpolation interval
 * @param time time being interpolated to
 */
 inline ImuMsg interpolateImu(const ImuMsg &imu_beg, const ImuMsg &imu_end, double time) {
  // time-distance lambda
  double lambda = (time - imu_beg.time) / (imu_end.time - imu_beg.time);
  // cout << "lambda - " << lambda << endl;
  // interpolate between the two times
  ImuMsg data;
  data.time = time;
  data.a = (1 - lambda) * imu_beg.a + lambda * imu_end.a;
  data.w = (1 - lambda) * imu_beg.w + lambda * imu_end.w;
  return data;
}

 inline PressureMsg interpolatePressure(const PressureMsg &msg_beg, const PressureMsg &msg_end, double time) {
  // time-distance lambda
  double lambda = (time - msg_beg.time) / (msg_end.time - msg_beg.time);
  // cout << "lambda - " << lambda << endl;
  // interpolate between the two times
  PressureMsg data;
  data.time = time;
  data.p = (1 - lambda) * msg_beg.p + lambda * msg_end.p;
  return data;
}

 inline DvlMsg interpolateDvl(const DvlMsg &msg_beg, const DvlMsg &msg_end, double time) {
  // time-distance lambda
  double lambda = (time - msg_beg.time) / (msg_end.time - msg_beg.time);
  // interpolate between the two times
  DvlMsg data;
  data.time = time;
  data.v = (1 - lambda) * msg_beg.v + lambda * msg_end.v;
  return data;
}

} // namespace msckf_dvio

#endif  //MSCKF_UTILS_H_