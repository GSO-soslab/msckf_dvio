#ifndef MSCKF_UTILS_H_
#define MSCKF_UTILS_H_

#include <Eigen/Eigen>
#include "types/msgs.h"
#include <magic_enum.hpp>


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
 * @brief SO(3) matrix logarithm
 *
 * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 17 & 18.
 * http://ethaneade.com/lie.pdf
 * \f{align*}{
 * \theta &= \textrm{arccos}(0.5(\textrm{trace}(\mathbf{R})-1)) \\
 * \lfloor\mathbf{v}\times\rfloor &= \frac{\theta}{2\sin{\theta}}(\mathbf{R}-\mathbf{R}^\top)
 * @f}
 *
 * @param[in] R 3x3 SO(3) rotation matrix
 * @return 3x1 in the se(3) space [omegax, omegay, omegaz]
 */
inline Eigen::Matrix<double, 3, 1> logSO3(const Eigen::Matrix<double, 3, 3> &R) {
  // magnitude of the skew elements (handle edge case where we sometimes have a>1...)
  double a = 0.5 * (R.trace() - 1);
  double theta = (a > 1) ? acos(1) : ((a < -1) ? acos(-1) : acos(a));
  // Handle small angle values
  double D;
  if (theta < 1e-12) {
    D = 0.5;
  } else {
    D = theta / (2 * sin(theta));
  }
  // calculate the skew symetric matrix
  Eigen::Matrix<double, 3, 3> w_x = D * (R - R.transpose());
  // check if we are near the identity
  if (R != Eigen::MatrixXd::Identity(3, 3)) {
    Eigen::Vector3d vec;
    vec << w_x(2, 1), w_x(0, 2), w_x(1, 0);
    return vec;
  } else {
    return Eigen::Vector3d::Zero();
  }
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



inline std::tuple<Eigen::Matrix3d, Eigen::Vector3d> interpolatePose(
  Eigen::VectorXd &pose_a, double t_a,
  Eigen::VectorXd &pose_b, double t_b,
  double time) {

  // prepare the transformation
  Eigen::Matrix3d R_Ia_G = toRotationMatrix(pose_a.block(0,0,4,1));
  Eigen::Vector3d p_G_Ia = pose_a.block(4,0,3,1);

  Eigen::Matrix3d R_Ib_G = toRotationMatrix(pose_b.block(0,0,4,1));
  Eigen::Vector3d p_G_Ib = pose_b.block(4,0,3,1);

  // calculate the lambda
  double lambda = (time - t_a) / (t_b - t_a);

  // do interpolation
  Eigen::Matrix3d R = expSO3( lambda * logSO3(R_Ib_G * R_Ia_G.transpose())) * R_Ia_G;
  Eigen::Vector3d p = (1 - lambda) * p_G_Ia + lambda * p_G_Ib;

  return std::make_tuple(R, p);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

template <typename T>
std::string toCloneStamp(const T value, const int n = 9)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << value;
    return out.str();
}

inline double toSensorStamp(const std::string value) {
  return std::stod(value);
}

template <typename T>
inline std::string enumToString (const T enum_name)
{
    return static_cast<std::string>(magic_enum::enum_name(enum_name));
}

template <typename T>
inline T stringToEnum(const std::string &str, const T enum_none) {

  return magic_enum::enum_cast<T>(str).value_or(enum_none);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// from https://stackoverflow.com/a/2922778
inline int pointInPolygon( const std::vector<Eigen::Vector2d> &poly, const Eigen::Vector2d &test )
{
    // There must be at least 3 vertices 
    if (poly.size() < 3) {
        return 0;
    } 

    // check the input is vaild
    if(test.isZero(0)) {
        return 0;
    }

    int i, j, c = 0;
    for (i = 0, j = poly.size()-1; i < poly.size(); j = i++) {
      if ( ((poly[i].y()>test.y()) != (poly[j].y()>test.y())) &&
        (test.x() < (poly[j].x()-poly[i].x()) * (test.y()-poly[i].y()) / 
                    (poly[j].y()-poly[i].y()) + poly[i].x()) )
        c = !c;
    }
    return c;
} 


// https://www.particleincell.com/2012/quad-interpolation/
inline bool bilinearInterpolation(std::vector<Eigen::Vector3d> &quadrilateral, Eigen::Vector2d &feat, double  &depth) {
  //==================== Determine the mapping function ====================//
  // x = alpha(0) + alpha(1) * l + alpha(2) * m + alpha(3) * l * m
  // y = beta(0) + beta(1) * l + beta(2) * m + beta(3) * l * m
  // Alpha = mat * X
  // Beta = mat * Y

  // get function matrix
  Eigen::Matrix4d mat;
  mat << 1,  0,  0,  0,
        -1,  1,  0,  0,
        -1,  0,  0,  1,
         1, -1,  1, -1;

  // get physical coordinate
  Eigen::Vector4d x, y;
  for(size_t i=0; i < quadrilateral.size(); i++) {
    x(i) = quadrilateral.at(i)(0);
    y(i) = quadrilateral.at(i)(1);
  }
  // std::cout<<"x: " << x.transpose() <<std::endl;
  // std::cout<<"y: " << y.transpose() <<std::endl;

  // get coefficients
  Eigen::Vector4d alpha, beta;
  alpha = mat * x;
  beta = mat * y;
  // std::cout<<"alpha: " << alpha.transpose() <<std::endl;
  // std::cout<<"beta: " << beta.transpose() <<std::endl;

  //==================== Map feature from physical coordinate to logical coordinate  ====================//

  // construct quadratic equation
  // (α3β2 – α2β3)m^2+(α3β0 − α0β3 + α1β2 − α2β1 + xβ3 – yα3)m + (α1β0 – α0β1 + xβ1 – yα1)=0
  // l = (x - alpha(0) - alpha(2) * m) / (alpha(1) + alpha(3) * m)
  double a = alpha(3) * beta(2) - alpha(2) * beta(3);
  double b = alpha(3) * beta(0) - alpha(0) * beta(3) + alpha(1) * beta(2) - 
             alpha(2) * beta(1) + feat(0) * beta(3) - feat(1) * alpha(3);
  double c = alpha(1) * beta(0) - alpha(0) * beta(1) +
             feat(0) * beta(1) - feat(1) * alpha(1);
  double delta = b * b - 4 * a * c;

  // get soultion
  double m,l;
  std::vector<Eigen::Vector2d> square;
  square.push_back(Eigen::Vector2d(0,0));
  square.push_back(Eigen::Vector2d(1,0));
  square.push_back(Eigen::Vector2d(1,1));
  square.push_back(Eigen::Vector2d(0,1));

  if(delta < 0) {
    // no solution
    // std::cout<<"No Solution: delat < 0\n";
    return false;
  }
  else if (delta ==0) {
    // 1 solution
    m = (- b ) / (2 * a);
    l = (feat(0) - alpha(0) - alpha(2) * m) / (alpha(1) + alpha(3) * m);

    // check if inside square
    if(!pointInPolygon(square, Eigen::Vector2d(l,m))) {
      // std::cout<<"One Solution: not inside square\n";
      return false;
    }
  }
  else {
    // 2 solutions
    double m_1 =  (- b + sqrt(delta)) / (2 * a);
    double m_2 =  (- b - sqrt(delta)) / (2 * a);

    double l_1 = (feat(0) - alpha(0) - alpha(2) * m_1) / (alpha(1) + alpha(3) * m_1);
    double l_2 = (feat(0) - alpha(0) - alpha(2) * m_2) / (alpha(1) + alpha(3) * m_2);

    // check if inside square
    bool solution_1 = pointInPolygon(square, Eigen::Vector2d(l_1,m_1)) ? true : false;
    bool solution_2 = pointInPolygon(square, Eigen::Vector2d(l_2,m_2)) ? true : false;

    if(!solution_1 && !solution_2) {
      // std::cout<<"Two Solution: not inside square\n";
      return false;
    }
    else if(solution_1 && solution_2) {
      // std::cout<<"Two Solution: both inside square\n";
      return false;
    }
    else {
      if(solution_1) {
        m = m_1;
        l = l_1;
      }
      else {
        m = m_2;
        l = l_2;
      }
    }
  }

  // std::cout<<" l=" << l <<", m=" << m<<std::endl;

  //==================== Bilinear Interpolation the Square  ====================//

  Eigen::MatrixXd l_side(1,2);
  Eigen::MatrixXd m_side(2,1);
  Eigen::MatrixXd depth_mat(2,2);
  l_side(0,0) = 1 - l;
  l_side(0,1) = l;
  m_side(0,0) = 1 - m;
  m_side(1,0) = m;
  depth_mat(0,0) = quadrilateral[0](2);
  depth_mat(1,0) = quadrilateral[1](2);
  depth_mat(1,1) = quadrilateral[2](2);
  depth_mat(0,1) = quadrilateral[3](2);

  Eigen::MatrixXd interpolated = l_side * depth_mat * m_side;
  depth = interpolated(0,0);
  // std::cout<<"Given 4 depth: "<<quadrilateral[0](2)<<", "<< quadrilateral[1](2) <<","
  //                             <<quadrilateral[2](2)<<", "<< quadrilateral[3](2) <<std::endl;
  // std::cout<<"Estimated depth= " << depth <<std::endl;

  return true;
}

} // namespace msckf_dvio

#endif  //MSCKF_UTILS_H_