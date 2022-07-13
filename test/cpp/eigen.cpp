#include <Eigen/Eigen>
#include <iostream>
#include <iomanip>

#define PI 3.1415926

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
  //---------- TEST1: ----------//
  // Eigen::Vector4d q = Eigen::VectorXd::Zero(4, 1);
  // q(3) =1;

  // Eigen::Matrix3d r = toRotationMatrix(q);

  //---------- TEST2: ----------//

  Eigen::Matrix4d T_C_I;
  T_C_I <<  -0.9688065230618990,	-0.1513897344668752,	0.1962016033847924, 10.8091237119852952,	
              0.0587324669980907,	-0.9094231784477713, -0.4117037524996234,	85.8174314765463890,
              0.2407580075366810,	-0.3873378767949271,	0.8899466000873116, 42.2335682983946370,
              0,0,0,1;


		
  std::cout<<std::setprecision(12);
  std::cout<<"T_C_I:\n" <<T_C_I<<std::endl;

  Eigen::Matrix4d T_I_C1;

  T_I_C1 = T_C_I.inverse();

  std::cout<<"T_I_C1:\n" << T_I_C1<<std::endl;


  // Eigen::Matrix4d T_I_C2 = Eigen::Matrix4d::Identity();

  // T_I_C2.block(0,0,3,3) = T_C_I.block(0,0,3,3).inverse();
  // T_I_C2.block(0,3,3,1) = - T_C_I.block(0,0,3,3).inverse() * T_C_I.block(0,3,3,1);
  // std::cout<<"T_I_C2:\n" << T_I_C2<<std::endl;

  //---------- TEST 3: ----------//

  // // create rotation matrix
  // Eigen::Matrix3d R;
  // R = Eigen::AngleAxisd(-3.142, Eigen::Vector3d::UnitZ()) * 
  //     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
  //     Eigen::AngleAxisd(-3.142, Eigen::Vector3d::UnitX());

  // Eigen::Quaterniond q(0,0,1,0);
  // Eigen::Matrix3d R_q = q.toRotationMatrix();

  // std::cout<<"R:\n"<<R<<std::endl;
  // std::cout<<"R_q:\n"<<R_q<<std::endl;
}

