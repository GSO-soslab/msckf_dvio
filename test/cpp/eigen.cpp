#include <Eigen/Eigen>


// Returns skew-symmetric form of a 3-d vector
inline Eigen::Matrix3d toSkewSymmetric(const Eigen::Vector3d& vec) {

  Eigen::Matrix3d mat;
  mat <<      0, -vec(2),  vec(1),
         vec(2),       0, -vec(0),
        -vec(1),  vec(0),       0;

  return mat;
}

inline Eigen::Matrix<double, 3, 3> toRotationMatrix(const Eigen::Matrix<double, 4, 1> &q) {
  printf("rot1\n");

  Eigen::Matrix<double, 3, 3> q_x = toSkewSymmetric(q.block(0, 0, 3, 1));
  printf("aaa\n");

  Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                        2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
  printf("bbb\n");

  return Rot;
}

int main()
{
  Eigen::Vector4d q = Eigen::VectorXd::Zero(4, 1);
  q(3) =1;


  Eigen::Matrix3d r = toRotationMatrix(q);
}