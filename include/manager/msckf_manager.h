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

// 3rd
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// customized
#include "types/type_all.h"

#include "utils/utils.h"
#include "utils/recorder.h"
#include "utils/time_cost.h"

#include "msckf/state.h"
#include "msckf/predictor.h"
#include "msckf/updater.h"

#include "initializer/initializer_dvl_aided.h"
#include "initializer/initializer_setting.h"

#include "tracker/TrackKLT.h"
#include "tracker/TrackFeature.h"

namespace msckf_dvio
{

class MsckfManager {

public:
  MsckfManager(Params &parameters);

  void feedImu(const ImuMsg &data);

  void feedDvl(const DvlMsg &data);

  void feedCamera(ImageMsg &data);

  void feedFeature(FeatureMsg &data);

  void feedPressure(const PressureMsg &data);

  void feedDvlCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data);

  void backend();

  bool isInitialized() { return initializer->isInit(); }

  std::shared_ptr<State> getState() { return state; }

  std::shared_ptr<TrackBase> getTracker() { return tracker; }

  void cleanFeatures() { trig_feat.clear(); }

  //! TODO: just for test, better handling in visulization_manager

  void setFeatures(std::vector<Feature> &features);

  std::vector<Eigen::Vector3d> getFeatures();

  bool isFeature() {return is_feat; }

  cv::Mat getImgHistory();

  // get the N pointclouds that are closet to the given timestamp
  std::vector<pcl::PointCloud<pcl::PointXYZ>> getSpareCloud(double timestamp, int num);

  void updateImgHistory();

  void setupTest(std::unordered_map<size_t, Eigen::Vector3d> &test_data) {
    truth_feature = test_data;
  }

private:

  void doDVL();

  void doDvlBT();

  void doPressure();

  void doCameraSlideWindow();

  void doCameraKeyframe();

  void doPressure_test();

  Sensor selectUpdateSensor();

  std::vector<ImuMsg> selectImu(double t_begin, double t_end);

  void getDataForPressure(PressureMsg &pressure, DvlMsg &dvl, std::vector<ImuMsg> &imus);

  bool checkMotion();

  bool checkScene(double curr_time);

  bool checkFeatures(double curr_time);

  bool checkAdaptive();

  bool checkFrameCount();

  void selectFeatures(const double time_update, std::vector<Feature> &feat_selected);

  void selectFeaturesSlideWindow(
    const double time_update, std::vector<Feature> &feat_lost, std::vector<Feature> &feat_marg);

  void selectFeaturesKeyFrame(
    const double time_update, std::vector<Feature> &feat_lost, std::vector<Feature> &feat_marg);

  void releaseImuBuffer(double timeline);

  void releaseDvlBuffer(double timeline);

  void releasePressureBuffer(double timeline);

  int frame_count;

  double frame_distance;

  std::vector<ImuMsg> buffer_imu;
  
  //! TODO: change dvl and pressure to queue
  std::vector<DvlMsg> buffer_dvl;
  std::vector<PressureMsg> buffer_pressure;
  std::queue<double> buffer_time_img;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> buf_dvl_pc;

  DvlMsg last_dvl; 
  
  //! TODO: add more mutex for different resource management
  std::mutex mtx;

  std::shared_ptr<Recorder> recorder;

  std::shared_ptr<State> state;

  std::shared_ptr<Initializer> initializer;

  std::shared_ptr<Predictor> predictor;

  std::shared_ptr<Updater> updater;

  std::shared_ptr<TrackBase> tracker;

  Params params;

  //! TEST: 
  const char *file_path="/home/lin/Desktop/msckf_manager.txt";
  std::ofstream file;
  // file.open(file_path, std::ios_base::app);//std::ios_base::app
  // file.close();

  std::unordered_map<size_t, Eigen::Vector3d> truth_feature;

  // get triangulated feature position
  std::atomic<bool> is_feat;
  std::vector<Eigen::Vector3d> trig_feat;

  std::queue<double> slide_window;
  cv::Mat img_history;
  std::recursive_mutex img_mtx;

};

} // namespace msckf_dvio


#endif // MSCKF_MANAGER_MSCKF_H

