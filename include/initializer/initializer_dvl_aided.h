#ifndef MSCKF_INITIALIZER_DVL_AIDED_H_
#define MSCKF_INITIALIZER_DVL_AIDED_H_

#include "initializer.h"

namespace msckf_dvio {

class InitDvlAided : public Initializer
{

public:
  InitDvlAided(paramInit param_init_, priorImu prior_imu_, priorDvl prior_dvl_);

  ~InitDvlAided(){}

  void checkInit() override;

  void updateInit(std::shared_ptr<State> state, const Params &params, std::vector<double> &data_time) override;

  void cleanBuffer() override;

protected:

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

  priorImu prior_imu;
  priorDvl prior_dvl;


  int last_index_imu, last_index_dvl;

  double align_time_imu, align_time_dvl;

  //// IMU data, variance for one section in each IMU window
  std::vector<std::tuple<std::vector<ImuMsg>, double>> sections_imu;
  std::vector<DvlMsg> sections_dvl;

  //! TODO: replace this with state update

  double time_I;
  Eigen::Vector4d q_I_G;
  Eigen::Vector3d p_G_I;
  Eigen::Vector3d v_G_I;
  Eigen::Vector3d bg_avg;  
  Eigen::Vector3d ba_avg;

  double time_D;
  double time_I_D;
};

}
#endif //MSCKF_INITIALIZER_DVL_AIDED_H_