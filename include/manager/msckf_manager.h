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
#include <queue>
#include <limits>
// customized
#include "types/type_all.h"

#include "utils/utils.h"

#include "msckf/state.h"
#include "msckf/predictor.h"
#include "msckf/updater.h"

#include "initializer/initializer_dvl_aided.h"

#include "tracker/TrackKLT.h"


namespace msckf_dvio
{

class MsckfManager {

public:
  MsckfManager(Params &parameters);

  void feedImu(const ImuMsg &data);

  void feedDvl(const DvlMsg &data);

  void feedCamera(ImageMsg &data);

  void feedPressure(const PressureMsg &data);

  void backend();

  bool isInitialized() { return initializer->isInit(); }

  //! TODO: just for test, better handling in visulization_manager
  //! 
  bool isOdom() { return is_odom; }

  void resetOdom() { is_odom = false;}

  Eigen::VectorXd getNewImuState() {return state->getImuValue(); }

  double getTime() {return state->getTimestamp(); }

  bool checkTrackedImg() {
    buffer_mutex.unlock();
    int size = tracked_img.size();
    buffer_mutex.unlock();

    return size > 0 ? true : false;
  }

  ImageMsg getTrackedImg() {
    buffer_mutex.unlock();
    ImageMsg img = tracked_img.front();
    tracked_img.pop();
    buffer_mutex.unlock();

    return img;
  }

private:

  std::vector<ImuMsg> selectImu(double t_begin, double t_end);

  void doDVL();

  void doPressure();

  void doCamera();

  UpdateSouce selectUpdateSource();

  std::vector<ImuMsg> buffer_imu;
  std::vector<DvlMsg> buffer_dvl;
  std::vector<PressureMsg> buffer_pressure;
  std::vector<double> buffer_img_time;

  std::mutex buffer_mutex;

  std::shared_ptr<State> state;

  std::shared_ptr<Initializer> initializer;

  std::shared_ptr<Predictor> predictor;

  std::shared_ptr<Updater> updater;

  std::shared_ptr<TrackBase> tracker;
  std::map<size_t, bool> camera_fisheye;
  std::map<size_t, Eigen::VectorXd> camera_calibration;

  Params params;

  //! TEST: 
  int test=0;
  std::atomic<bool> is_odom;
  const char *file_path="/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_data.dat";

  std::queue<ImageMsg> tracked_img;
};

} // namespace msckf_dvio


#endif // MSCKF_MANAGER_MSCKF_H

