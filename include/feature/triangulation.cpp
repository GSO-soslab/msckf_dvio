#include "triangulation.h"

namespace msckf_dvio
{

FeatureTriangulation::FeatureTriangulation(paramTriangulation param_trig) :
  param_trig_(param_trig) {}

double FeatureTriangulation::compute_error(
  Feature *feature, 
  const std::unordered_map<double, Eigen::Matrix4d> &T_G_C,
  double alpha, double beta, double rho) {
    
  // Total error
  double err = 0;

  // get Anchor transfomration
  Eigen::Matrix3d R_A_G = T_G_C.begin()->second.block(0,0,3,3).transpose();
  Eigen::Vector3d p_G_A = T_G_C.begin()->second.block(0,3,3,1);

  for(size_t i=0; i<feature->timestamps.at(0).size(); i++) {

    // get camera transform based on feature timestamp
    const Eigen::Matrix<double, 3, 3> &R_G_Ci = 
        T_G_C.at(feature->timestamps.at(0).at(i)).block(0,0,3,3);
    const Eigen::Matrix<double, 3, 1> &p_G_Ci = 
        T_G_C.at(feature->timestamps.at(0).at(i)).block(0,3,3,1);

    // transfrom camera into anchor frame(first camera frame)
    Eigen::Matrix<double, 3, 3> R_A_Ci = R_A_G * R_G_Ci;
    Eigen::Matrix<double, 3, 1> p_A_Ci = R_A_G * (p_G_Ci - p_G_A);
    Eigen::Matrix<double, 3, 3> R_Ci_A = R_A_Ci.transpose();
    Eigen::Matrix<double, 3, 1> p_Ci_A = - R_Ci_A * p_A_Ci;

    // Middle variables of the system
    double hi1 = R_Ci_A(0, 0) * alpha + R_Ci_A(0, 1) * beta + R_Ci_A(0, 2) + rho * p_Ci_A(0, 0);
    double hi2 = R_Ci_A(1, 0) * alpha + R_Ci_A(1, 1) * beta + R_Ci_A(1, 2) + rho * p_Ci_A(1, 0);
    double hi3 = R_Ci_A(2, 0) * alpha + R_Ci_A(2, 1) * beta + R_Ci_A(2, 2) + rho * p_Ci_A(2, 0);

    // Calculate residual
    Eigen::Matrix<float, 2, 1> z;
    z << hi1 / hi3, hi2 / hi3;
    Eigen::Matrix<float, 2, 1> res = feature->uvs_norm.at(0).at(i) - z;
    // Append to our summation variables
    err += pow(res.norm(), 2);
  }

  return err;
}

//! @param feature: each tracked feature 
//! @param T_G_C: transform of Camera with repect to Global frame
//!
bool FeatureTriangulation::single_triangulation(
  Feature *feature, 
  const std::unordered_map<double, Eigen::Matrix4d> &T_G_C) {        

  // get Anchor transfomration
  Eigen::Matrix3d R_A_G = T_G_C.begin()->second.block(0,0,3,3).transpose();
  Eigen::Vector3d p_G_A = T_G_C.begin()->second.block(0,3,3,1);

  // Construct linear system matrices
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  for(size_t i=0; i<feature->timestamps.at(0).size(); i++) {

    // get camera transform based on feature timestamp
    // printf("T_G_C s: %ld, feature s:%ld, i:%ld, t:%f\n",
    //        T_G_C.size(), feature->timestamps.at(0).size(), i, feature->timestamps.at(0).at(i));
    // std::cout<<"T: \n"<<T_G_C.at(feature->timestamps.at(0).at(i))<<std::endl;

    const Eigen::Matrix<double, 3, 3> &R_G_Ci = 
        T_G_C.at(feature->timestamps.at(0).at(i)).block(0,0,3,3);
    const Eigen::Matrix<double, 3, 1> &p_G_Ci = 
        T_G_C.at(feature->timestamps.at(0).at(i)).block(0,3,3,1);

    // transfrom camera into anchor frame(first camera frame)
    Eigen::Matrix<double, 3, 3> R_A_Ci = R_A_G * R_G_Ci;
    Eigen::Matrix<double, 3, 1> p_A_Ci = R_A_G * (p_G_Ci - p_G_A);

    // Get the UV coordinate normal
    Eigen::Matrix<double, 3, 1> f_A;
    f_A << feature->uvs_norm.at(0).at(i)(0), feature->uvs_norm.at(0).at(i)(1), 1;
    f_A = R_A_Ci * f_A;
    f_A = f_A / f_A.norm();
    Eigen::Matrix3d Bperp = toSkewSymmetric(f_A);

    // Append to our linear system
    Eigen::Matrix3d Ai = Bperp.transpose() * Bperp;
    A += Ai;
    b += Ai * p_A_Ci;
  }

  // Solve the linear system
  Eigen::MatrixXd p_A_F = A.colPivHouseholderQr().solve(b);

  // Check A and p_A_F
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // printf("row:%d, col:%d\n",svd.singularValues().rows(), svd.singularValues().cols());

  Eigen::Vector3d singular = svd.singularValues();
  double condA = singular(0) / singular(singular.rows() - 1);
  // printf("singular: %f,%f,%f\n", singular(0),singular(1),singular(2));
  // printf("conA:%f\n", condA);

  // If we have a bad condition number, or it is too close
  // Then set the flag for bad (i.e. set z-axis to nan)
  if (std::isnan(p_A_F.norm()) || p_A_F(2) < param_trig_.min_dist || 
      p_A_F(2) > param_trig_.max_dist || std::abs(condA) > param_trig_.max_cond_number) {
    return false;
  }

  // Store it in our feature object
  feature->p_FinA = p_A_F;
  feature->p_FinG = R_A_G.transpose() * feature->p_FinA + p_G_A;
  // printf("tri 1: x:%f,y:%f,z:%f\n", feature->p_FinG(0), feature->p_FinG(1), feature->p_FinG(2) );

  return true;
  
}

//! @param feature: each tracked feature 
//! @param T_G_C: transform of Camera with repect to Global frame
//!
bool FeatureTriangulation::single_gaussnewton(
  Feature *feature, 
  const std::unordered_map<double, Eigen::Matrix4d> &T_G_C) {

  // get Anchor transfomration
  Eigen::Matrix3d R_A_G = T_G_C.begin()->second.block(0,0,3,3).transpose();
  Eigen::Vector3d p_G_A = T_G_C.begin()->second.block(0,3,3,1);

  // Get into inverse depth
  double alpha = feature->p_FinA(0) / feature->p_FinA(2);
  double beta = feature->p_FinA(1) / feature->p_FinA(2);
  double rho = 1 / feature->p_FinA(2);

  // Optimization parameters
  double lam = 1e-3;  /// Init lambda for Levenberg-Marquardt optimization
  double eps = 10000;
  int runs = 0;

  // Variables used in the optimization
  bool recompute = true;
  Eigen::Matrix<double, 3, 3> Hess = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 1> grad = Eigen::Matrix<double, 3, 1>::Zero();

  // Cost at the last iteration
  double cost_old = compute_error(feature, T_G_C, alpha, beta, rho);

  // Loop till we have either
  // 1. Reached our max iteration count
  // 2. System is unstable
  // 3. System has converged
  while (runs < param_trig_.max_runs && 
         lam < param_trig_.max_lamda && 
         eps > param_trig_.min_dx) {
    // Triggers a recomputation of jacobians/information/gradients
    if (recompute) {

      Hess.setZero();
      grad.setZero();

      double err = 0;

      // loop cam pose for this feature
      for(size_t i=0; i<feature->timestamps.at(0).size(); i++) {

        //=====================================================================================

        // get camera transform based on feature timestamp
        const Eigen::Matrix<double, 3, 3> &R_G_Ci = 
            T_G_C.at(feature->timestamps.at(0).at(i)).block(0,0,3,3);
        const Eigen::Matrix<double, 3, 1> &p_G_Ci = 
            T_G_C.at(feature->timestamps.at(0).at(i)).block(0,3,3,1);

        // transfrom camera into anchor frame(first camera frame)
        Eigen::Matrix<double, 3, 3> R_A_Ci = R_A_G * R_G_Ci;
        Eigen::Matrix<double, 3, 1> p_A_Ci = R_A_G * (p_G_Ci - p_G_A);
        Eigen::Matrix<double, 3, 3> R_Ci_A = R_A_Ci.transpose();
        Eigen::Matrix<double, 3, 1> p_Ci_A = - R_Ci_A * p_A_Ci;

        //=====================================================================================

        // Middle variables of the system
        double hi1 = R_Ci_A(0, 0) * alpha + R_Ci_A(0, 1) * beta + R_Ci_A(0, 2) + rho * p_Ci_A(0, 0);
        double hi2 = R_Ci_A(1, 0) * alpha + R_Ci_A(1, 1) * beta + R_Ci_A(1, 2) + rho * p_Ci_A(1, 0);
        double hi3 = R_Ci_A(2, 0) * alpha + R_Ci_A(2, 1) * beta + R_Ci_A(2, 2) + rho * p_Ci_A(2, 0);
        // Calculate jacobian
        double d_z1_d_alpha = (R_Ci_A(0, 0) * hi3 - hi1 * R_Ci_A(2, 0)) / (pow(hi3, 2));
        double d_z1_d_beta = (R_Ci_A(0, 1) * hi3 - hi1 * R_Ci_A(2, 1)) / (pow(hi3, 2));
        double d_z1_d_rho = (p_Ci_A(0, 0) * hi3 - hi1 * p_Ci_A(2, 0)) / (pow(hi3, 2));
        double d_z2_d_alpha = (R_Ci_A(1, 0) * hi3 - hi2 * R_Ci_A(2, 0)) / (pow(hi3, 2));
        double d_z2_d_beta = (R_Ci_A(1, 1) * hi3 - hi2 * R_Ci_A(2, 1)) / (pow(hi3, 2));
        double d_z2_d_rho = (p_Ci_A(1, 0) * hi3 - hi2 * p_Ci_A(2, 0)) / (pow(hi3, 2));
        Eigen::Matrix<double, 2, 3> H;
        H << d_z1_d_alpha, d_z1_d_beta, d_z1_d_rho, d_z2_d_alpha, d_z2_d_beta, d_z2_d_rho;
        // Calculate residual
        Eigen::Matrix<float, 2, 1> z;
        z << hi1 / hi3, hi2 / hi3;
        Eigen::Matrix<float, 2, 1> res = feature->uvs_norm.at(0).at(i) - z;

        //===================================================================================== //

        // Append to our summation variables
        err += std::pow(res.norm(), 2);
        grad.noalias() += H.transpose() * res.cast<double>();
        Hess.noalias() += H.transpose() * H;
      }
    } // end of computing jacobians

    // Solve Levenberg iteration
    Eigen::Matrix<double, 3, 3> Hess_l = Hess;
    for (size_t r = 0; r < (size_t)Hess.rows(); r++)
      Hess_l(r, r) *= (1.0 + lam);
    Eigen::Matrix<double, 3, 1> dx = Hess_l.colPivHouseholderQr().solve(grad);

    // Check if error has gone down
    double cost = compute_error(feature, T_G_C, alpha + dx(0, 0), beta + dx(1, 0), rho + dx(2, 0));
    //// TEST: print
    // cout << "run = " << runs << " | cost = " << dx.norm() << " | lamda = " << lam << " | depth = " << 1/rho << endl;

    // Check if converged
    if (cost <= cost_old && (cost_old - cost) / cost_old < param_trig_.min_dcost) {
      alpha += dx(0, 0);
      beta += dx(1, 0);
      rho += dx(2, 0);
      eps = 0;
      break;
    }

    // If cost is lowered, accept step
    // Else inflate lambda (try to make more stable)
    if (cost <= cost_old) {
      recompute = true;
      cost_old = cost;
      alpha += dx(0, 0);
      beta += dx(1, 0);
      rho += dx(2, 0);
      runs++;
      lam = lam / param_trig_.lam_mult;
      eps = dx.norm();
    } else {
      recompute = false;
      lam = lam * param_trig_.lam_mult;
      continue;
    }

  } // end of optimization

  // Revert to standard, and set to all
  feature->p_FinA(0) = alpha / rho;
  feature->p_FinA(1) = beta / rho;
  feature->p_FinA(2) = 1 / rho;

  // Get tangent plane to x_hat
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(feature->p_FinA);
  Eigen::MatrixXd Q = qr.householderQ();

  // loop cam pose for this feature to check baseline
  // Max baseline we have between poses
  double base_line_max = 0.0;

  for(size_t i=0; i<feature->timestamps.at(0).size(); i++) {

    // get camera transform based on feature timestamp
    const Eigen::Matrix<double, 3, 1> &p_G_Ci = 
        T_G_C.at(feature->timestamps.at(0).at(i)).block(0,3,3,1);

    // transfrom camera into anchor frame(first camera frame)
    Eigen::Matrix<double, 3, 1> p_A_Ci = R_A_G * (p_G_Ci - p_G_A);


    // Dot product camera pose and nullspace
    double base_line = ((Q.block(0, 1, 3, 2)).transpose() * p_A_Ci).norm();
    if (base_line > base_line_max)
      base_line_max = base_line;
  }

  // Check if this feature is bad or not
  // 1. If the feature is too close
  // 2. If the feature is invalid
  // 3. If the baseline ratio is large

  if (feature->p_FinA(2) < param_trig_.min_dist ||
      feature->p_FinA(2) > param_trig_.max_dist ||
      (feature->p_FinA.norm() / base_line_max) > param_trig_.max_baseline || 
      std::isnan(feature->p_FinA.norm())) {
    return false;
  }

  feature->p_FinG = R_A_G.transpose() * feature->p_FinA + p_G_A;
  feature->triangulated = true;

  // printf("tri 2 : x:%f,y:%f,z:%f, base:%f\n", 
  //   feature->p_FinG(0), feature->p_FinG(1), feature->p_FinG(2), feature->p_FinA.norm() / base_line_max);

  return true;
}


} // namespace msckf_dvio