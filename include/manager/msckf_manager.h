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
#include <chrono>
// customized
#include "types/type_all.h"

#include "utils/utils.h"
#include "utils/recorder.h"

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

  std::shared_ptr<State> getState() { return state; }

  std::shared_ptr<TrackBase> getTracker() { return tracker; }

  std::vector<Eigen::Vector3d> getFeatures() { return trig_feat; }

  void cleanFeatures() { trig_feat.clear(); }

  //! TODO: just for test, better handling in visulization_manager

  void setFeaturesTest2(std::vector<Feature> &features);

  std::vector<Eigen::Vector3d> getFeaturesTest1();

  bool isFeature() {return is_feat; }

private:

  void doDVL();

  void doDvlBT();

  void doPressure();

  void doCamera();

  void doPressure_test();

  SensorName selectUpdateSensor();

  std::vector<ImuMsg> selectImu(double t_begin, double t_end);

  std::vector<std::shared_ptr<Feature>> selectFeatures(const double time_curr);

  void selectFeaturesTest(const double time_update, std::vector<Feature> &feat_selected);

  void getDataForPressure(PressureMsg &pressure, DvlMsg &dvl, std::vector<ImuMsg> &imus);

  std::vector<ImuMsg> buffer_imu;
  
  //! TODO: change dvl and pressure to queue
  std::vector<DvlMsg> buffer_dvl;
  std::vector<PressureMsg> buffer_pressure;
  std::queue<double> buffer_time_img;

  DvlMsg last_dvl; 
  
  std::mutex mtx;

  std::shared_ptr<Recorder> recorder;

  std::shared_ptr<State> state;

  std::shared_ptr<Initializer> initializer;

  std::shared_ptr<Predictor> predictor;

  std::shared_ptr<Updater> updater;

  std::shared_ptr<TrackBase> tracker;
  std::map<size_t, bool> camera_fisheye;
  std::map<size_t, Eigen::VectorXd> camera_calibration;

  Params params;

  //! TEST: 
  
  const char *file_path="/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_data.dat";

  // get triangulated feature position
  std::atomic<bool> is_feat;
  std::vector<Eigen::Vector3d> trig_feat;

};

} // namespace msckf_dvio


#endif // MSCKF_MANAGER_MSCKF_H

