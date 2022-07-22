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
    std::unique_lock<std::mutex> lck(mtx);

    int size = tracked_img.size();

    return size > 0 ? true : false;
  }

  ImageMsg getTrackedImg() {
    std::unique_lock<std::mutex> lck(mtx);

    ImageMsg img = tracked_img.front();
    tracked_img.pop();

    return img;
  }

private:

  void doDVL();

  void doDvlBT();

  void doPressure();

  void doCamera();

  void doPressure_test();

  SensorName selectUpdateSensor();

  std::vector<ImuMsg> selectImu(double t_begin, double t_end);

  std::vector<std::shared_ptr<Feature>> selectFeatures(const double time_curr);

  void getDataForPressure(PressureMsg &pressure, DvlMsg &dvl, std::vector<ImuMsg> &imus);

  std::vector<ImuMsg> buffer_imu;
  
  //! TODO: change dvl and pressure to queue
  std::vector<DvlMsg> buffer_dvl;
  std::vector<PressureMsg> buffer_pressure;
  std::queue<double> buffer_time_img;

  DvlMsg last_dvl; 
  
  std::mutex mtx;

  std::shared_ptr<State> state;

  std::shared_ptr<Initializer> initializer;

  std::shared_ptr<Predictor> predictor;

  std::shared_ptr<Updater> updater;

  std::shared_ptr<TrackBase> tracker;
  std::map<size_t, bool> camera_fisheye;
  std::map<size_t, Eigen::VectorXd> camera_calibration;

  Params params;

  //! TEST: 
  int exe_counts=0;
  std::atomic<bool> is_odom;
  const char *file_path="/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_data.dat";

  std::queue<ImageMsg> tracked_img;
};

} // namespace msckf_dvio


#endif // MSCKF_MANAGER_MSCKF_H

