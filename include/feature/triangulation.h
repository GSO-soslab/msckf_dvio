#ifndef MSCKF_FEATURE_TRIANGULATION_H
#define MSCKF_FEATURE_TRIANGULATION_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <cassert>
#include "utils/utils.h"
#include "types/parameters.h"
#include "Feature.h"

namespace msckf_dvio
{

class FeatureTriangulation {

public:
  FeatureTriangulation(paramTriangulation param_trig);

  double compute_error(Feature *feature, 
                       const std::unordered_map<double, Eigen::Matrix4d> &T_G_C,
                       double alpha, double beta, double rho);

  bool single_triangulation(Feature *feature, 
                            const std::unordered_map<double, Eigen::Matrix4d> &T_G_C);

  bool single_gaussnewton(Feature *feature, 
                          const std::unordered_map<double, Eigen::Matrix4d> &T_G_C);

private:

  paramTriangulation param_trig_;

  const char *file_path="/home/lin/Desktop/msckf_manager.txt";
  std::ofstream file; 
};

}

#endif // MSCKF_FEATURE_TRIANGULATION_H