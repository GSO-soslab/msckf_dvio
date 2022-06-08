#ifndef MSCKF_MANAGER_MSCKF_H
#define MSCKF_MANAGER_MSCKF_H
//c++
#include <memory>
#include <mutex>
#include <algorithm>
#include <iterator>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <tuple>
#include <fstream>
// customized
#include "types/type_all.h"

#include "utils/utils.h"

#include "msckf/state.h"
#include "msckf/predictor.h"
#include "msckf/updater.h"

#include "initializer/imu_initializer.h"

namespace msckf_dvio
{

class MsckfManager {

public:
  MsckfManager(Params &parameters);

  void feedImu(const ImuMsg &data);

  void feedDvl(const DvlMsg &data);

  void feedCamera(const ImageMsg &data);

  void backend();

  bool checkNewDvl(); 

  bool checkNewImu();
  
  bool isInitialized() { return imu_initializer->isInitialized(); }

  //! TODO: just for test, better hanlding in visulization_manager
  bool isOdom() { return is_odom; }

  void resetOdom() { is_odom = false;}

  Eigen::VectorXd getNewImuState() {return state->getImuValue(); }

  double getTime() {return state->getTimestamp(); }

private:

  std::vector<ImuMsg> selectImu(double t_begin, double t_end);

  std::vector<ImuMsg> buffer_imu;
  std::vector<DvlMsg> buffer_dvl;
  std::vector<ImageMsg> buffer_img;

  std::mutex buffer_mutex;

  std::shared_ptr<State> state;

  std::shared_ptr<ImuInitializer> imu_initializer;

  std::shared_ptr<Predictor> predictor;

  std::shared_ptr<Updater> updater;

  Params params;

  //! TEST: 
  int test=0;
  std::atomic<bool> is_odom;
  const char *file_path="/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_data.dat";
};

} // namespace msckf_dvio


#endif // MSCKF_MANAGER_MSCKF_H

