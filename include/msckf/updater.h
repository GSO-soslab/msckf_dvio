#ifndef MSCKF_MSCKF_UPDATER_H
#define MSCKF_MSCKF_UPDATER_H

#include "types/type_all.h"
#include "utils/utils.h"
#include "utils/time_cost.h"
#include "msckf/state.h"

#include "feature/Feature.h"
#include "feature/triangulation.h"
#include <fstream>

#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>

namespace msckf_dvio
{

class Updater {

public:
  Updater(Params &params);

  void updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D);

  void updateDvl(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D, bool is_simple);

  void updatePressure(std::shared_ptr<State> state, const double pres_begin, const double pres_curr, bool is_simple);

  void updatePressureTest(std::shared_ptr<State> state, const double pres_begin, const double pres_curr);
  
  void marginalizeDvl(std::shared_ptr<State> state);

  void marginalize(std::shared_ptr<State> state, SubStateName clone_name);

  void updateCam(std::shared_ptr<State> state, std::vector<Feature> &features, double timestamp);
  
  void featureJacobian(std::shared_ptr<State> state, const Feature &feature, 
                       Eigen::MatrixXd &H_x, Eigen::MatrixXd &H_f, 
                       Eigen::VectorXd & res);
                       
  void nullspace_project(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  void nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  bool gatingTest(const Eigen::MatrixXd &H, const Eigen::VectorXd &r, const int &dof);

  void compress(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  void compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

private:
  priorDvl prior_dvl_;

  priorCam prior_cam_;

  paramMsckf param_msckf_;

  std::unique_ptr<FeatureTriangulation> triangulater;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;
  
  //! TEST:
  long long int count;
  const char *file_path="/home/lin/Desktop/msckf_dvio.txt";
  std::ofstream file;
  // file.open(file_path, std::ios_base::app);//std::ios_base::app
  // file<<"\n"<<"aaa "<<Value;
  // file.close();

};

} // end of namespace msckf_dvio 

#endif // MSCKF_MSCKF_UPDATER_H
