#ifndef MSCKF_INITIALIZER_IMU_H
#define MSCKF_INITIALIZER_IMU_H

#include "types/type_all.h"
#include "utils/utils.h"
#include <mutex>
#include <tuple>
#include <iomanip>
#include <cassert>

namespace msckf_dvio {

class ImuInitializer {

public:
  ImuInitializer(paramInit param_init_, 
                 priorImu prior_imu_, 
                 const Eigen::VectorXd &q_I_D_, 
                 const Eigen::Vector3d &p_I_D_);

  void feedImu(const ImuMsg &data);

  void feedDvl(const DvlMsg &data);

  bool isInitialized() { return is_initialized; }

  void checkInitialization();

  std::tuple<double, Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, double, double> 
    getInitResult() { return std::make_tuple(time_I, q_I_G, v_I, bg_avg, ba_avg, time_D, time_I_D); }

private:

  void findAlignmentImu();

  void findAlignmentDvl();

  bool grabInitializationData(std::vector<DvlMsg> &dvl_a, 
                              std::vector<ImuMsg> &imu_a,
                              std::vector<ImuMsg> &imu_g);

  void linearInterp(const std::vector<ImuMsg> &imu_in,
                    const std::vector<DvlMsg> &dvl_in, 
                          std::vector<DvlMsg> &dvl_out);

  void doInitialization(const std::vector<DvlMsg> &dvl_a, 
                        const std::vector<ImuMsg> &imu_a,
                        const std::vector<ImuMsg> &imu_g);

  void cleanBuffer();

  priorImu prior_imu;

  paramInit param_init;

  bool is_initialized;

  std::vector<ImuMsg> buffer_imu;
  std::vector<DvlMsg> buffer_dvl;

  std::mutex buffer_mutex;

  int last_index_imu, last_index_dvl;

  //// IMU data, variance for one section in each IMU window
  std::vector<std::tuple<std::vector<ImuMsg>, double>> sections_imu;
  std::vector<DvlMsg> sections_dvl;

  double align_time_imu, align_time_dvl;

  Eigen::Matrix3d R_I_D;
  Eigen::Vector3d p_I_D;

  //// initialzied result
  double time_I;
  double time_D;
  Eigen::Vector4d q_I_G;
  Eigen::Vector3d v_I;
  Eigen::Vector3d bg_avg;  
  Eigen::Vector3d ba_avg;
  double time_I_D;

};

}

#endif // MSCKF_INITIALIZER_IMU_H