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

  void updateDvlSimple(std::shared_ptr<State> state, const Eigen::Vector3d &w_I, const Eigen::Vector3d &v_D, bool is_simple);

  void updatePressure(std::shared_ptr<State> state, const double pres_begin, const double pres_curr, bool is_simple);

  void updatePressureTest(std::shared_ptr<State> state, const double pres_begin, const double pres_curr);
  
  void updatePressureSimple(std::shared_ptr<State> state, const double pres_begin, const double pres_curr);

  void updateDvlPressure(  
      std::shared_ptr<State> state,const Eigen::Vector3d &w_I, 
      const Eigen::Vector3d &v_D,const double pres_begin, const double pres_curr);

  void updateDvlPressureSimple(
      std::shared_ptr<State> state,const Eigen::Vector3d &w_I, 
      const Eigen::Vector3d &v_D,const double pres_begin, const double pres_curr); 

  void marginalize(std::shared_ptr<State> state, Sensor clone_name, int index);

  void updateCam(std::shared_ptr<State> state, std::vector<Feature> &features);

  void updateCamPart(std::shared_ptr<State> state, std::vector<Feature> &features);
  
  void cameraMeasurement(    
    std::shared_ptr<State> state, 
    std::vector<Feature> &features);

  void featureTriangulation(
    std::shared_ptr<State> state, 
    std::vector<Feature> &feat_lost,
    std::vector<Feature> &feat_marg,
    std::vector<Feature> &feat_msckf
  );

  void featureJacobian(std::shared_ptr<State> state, const Feature &feature, 
                       Eigen::MatrixXd &H_x, Eigen::MatrixXd &H_f, 
                       Eigen::VectorXd & res);
                       
  void featureJacobianPart(
    std::shared_ptr<State> state, const Feature &feature, 
    Eigen::MatrixXd &H_x, Eigen::MatrixXd &H_f, 
    Eigen::VectorXd & res, std::vector<std::shared_ptr<Type>> &x_order);

  void nullspace_project(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  void nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  bool chiSquareTest(
    std::shared_ptr<State> state, const Eigen::MatrixXd &H_x, 
    const Eigen::VectorXd &r, std::vector<std::shared_ptr<Type>> x_order);

  void compress(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  void compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  void update(
    std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

private:
  priorDvl prior_dvl_;

  priorPressure prior_pressure_;

  priorCam prior_cam_;

  paramMsckf param_msckf_;

  std::unique_ptr<FeatureTriangulation> triangulater;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;
  
  //! TEST:
  long long int count;
  const char *file_path="/home/lin/Desktop/msckf_manager.txt";
  std::ofstream file;
  // file.open(file_path, std::ios_base::app);//std::ios_base::app
  // file<<"\n"<<"aaa "<<Value;
  // file.close();

};

} // end of namespace msckf_dvio 

#endif // MSCKF_MSCKF_UPDATER_H
