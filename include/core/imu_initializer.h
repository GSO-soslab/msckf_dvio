#ifndef MSCKF_CORE_IMU_INITIALIZER_H
#define MSCKF_CORE_IMU_INITIALIZER_H

#include "types/type.h"
#include "utils/utils.h"
#include <mutex>
#include <tuple>
#include <iomanip>
#include <cassert>

namespace msckf_dvio {

class ImuInitializer {

public:
  ImuInitializer(int window_imu_, int window_dvl_, double imu_delta_var_1_, double imu_delta_var_2_, double imu_delta_,
                 double dvl_delta_, double gravity_, Eigen::Matrix4d T_I_D_):
    is_initialized(false), last_index_imu(0), last_index_dvl(0), window_imu(window_imu_), window_dvl(window_dvl_),
    align_time_imu(-1), align_time_dvl(-1), imu_delta_var_1(imu_delta_var_1_), imu_delta_var_2(imu_delta_var_2_),
    imu_delta(imu_delta_),  dvl_delta(dvl_delta_), gravity(gravity_), T_I_D(T_I_D_)
  {}

  void feedImu(const ImuMsg &data);

  void feedDvl(const DvlMsg &data);
  
  void feedDvl(const std::vector<DvlMsg> &data);

  bool getInitializationStatus() { return is_initialized; }

  void checkInitialization();

private:

  void findAlignmentImu();

  void findAlignmentDvl();

  bool grabInitializationData(std::vector<DvlMsg> &dvl_a, 
                              std::vector<ImuMsg> &imu_a,
                              std::vector<ImuMsg> &imu_g);

  void linearInterp(const double time_offset, const std::vector<ImuMsg> &imu_in,
                    const std::vector<DvlMsg> &dvl_in, std::vector<DvlMsg> &dvl_out);

  void doInitialization(const std::vector<DvlMsg> &dvl_a, 
                        const std::vector<ImuMsg> &imu_a,
                        const std::vector<ImuMsg> &imu_g);

  // bool initWithDVL();

  bool is_initialized;

  std::vector<ImuMsg> buffer_imu;
  std::vector<DvlMsg> buffer_dvl;

  std::mutex buffer_mutex;

  int last_index_imu, last_index_dvl;

  int window_imu, window_dvl;

  //// IMU data, variance for one section in each IMU window
  std::vector<std::tuple<std::vector<ImuMsg>, double>> sections_imu;
  std::vector<DvlMsg> sections_dvl;

  double align_time_imu, align_time_dvl;

  double imu_delta_var_1, imu_delta_var_2, imu_delta;

  double dvl_delta;

  double gravity;

  Eigen::Matrix4d T_I_D;

};

}

#endif // MSCKF_CORE_IMU_INITIALIZER_H
