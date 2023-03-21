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
// #undef NDEBUG
// #include <cassert>
// 3rd
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

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

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> getFeatures();

  bool isFeature() {return is_feat; }

  cv::Mat getImgHistory();

  // get the N pointclouds that are closet to the given timestamp
  std::vector<pcl::PointCloud<pcl::PointXYZ>> getDvlCloud(double timestamp, int num);

  void setupTest(std::unordered_map<size_t, Eigen::Vector3d> &test_data) {
    truth_feature = test_data;
  }

  void addMatchedPointcloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    double timestamp,
    const std::string &frame) {

    tuple_pcs.push_back(std::make_tuple(timestamp, cloud, frame));
  }

  std::vector<std::tuple<double, pcl::PointCloud<pcl::PointXYZ>, std::string>> getMatchedPointcloud() {

    std::vector<std::tuple<double, pcl::PointCloud<pcl::PointXYZ>, std::string>> tupled;

    for(const auto& tuple : tuple_pcs) {
      tupled.push_back(std::make_tuple(std::get<0>(tuple), *std::get<1>(tuple), std::get<2>(tuple)));
    }

    // free memory
    std::vector<std::tuple<double, pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string>>().swap(tuple_pcs);

    return tupled;
  }
  
  // check if the system has visual measurements
  bool checkVisual() {
    auto find_cam0 = std::find(params.sys.sensors.begin(), 
                               params.sys.sensors.end(), 
                               Sensor::CAM0) != 
                               params.sys.sensors.end();
    auto find_cam0_feature = std::find(params.sys.sensors.begin(), 
                                       params.sys.sensors.end(), 
                                       Sensor::CAM0_FEATURE) != 
                                       params.sys.sensors.end(); 

    return find_cam0 || find_cam0_feature;                                       
  }

private:

  /**
   *  @brief get all the data need for pressure update, e.g. pressure, IMU
   * 
   *  @param pressure_msg: selected pressure data for this update
   *  @param imu_msgs: selected a series of IMU for propagation
   * 
   *  @return True: successfully get data; 
   *  @return False: failed to get data
   * 
   */
  bool getImuForPressure(PressureMsg &pressure_msg, std::vector<ImuMsg> &imu_msgs);

  /**
   *  @brief get all the data need for dvl update, e.g. dvl, IMU
   * 
   *  @param dvl_msg: selected dvl data for this update
   *  @param imu_msgs: selected a series of IMU for propagation
   * 
   *  @return True: successfully get data; 
   *  @return False: failed to get data
   * 
   */
  bool getImuForBt(DvlMsg &dvl_msg,  std::vector<ImuMsg> &imu_msgs);

  /**
   *  @brief get all the data need for dvl+pressure update, e.g. dvl, pressure, IMU
   * 
   *  @param dvl_msg: selected dvl data for this update
   *  @param pressure_msg: selected pressure data for this update
   *  @param imu_msgs: selected a series of IMU for propagation
   * 
   *  @return True: successfully get data; 
   *  @return False: failed to get data
   * 
   */
  bool getImuForBtPressure(DvlMsg &dvl_msg,  PressureMsg &pressure_msg, std::vector<ImuMsg> &imu_msgs);

  void doBtUpdate();

  void doPressureUpdate();

  void doBtPressureUpdate();

  void doDVL();

  void doCameraKeyframe();

  Sensor selectUpdateSensor();

  std::vector<ImuMsg> selectImu(double t_begin, double t_end);

  void getDataForPressure(PressureMsg &pressure, DvlMsg &dvl, std::vector<ImuMsg> &imus);

  bool checkMotion();

  bool checkScene(double curr_time);

  bool checkFeatures(double curr_time);

  bool checkAdaptive();

  bool checkFrameCount();

  void selectFeatures(const double time_update, std::vector<Feature> &feat_selected);

  void selectFeaturesKeyFrame(
    const double time_update, std::vector<Feature> &feat_keyframe);

  void selectMargMeasurement(std::vector<Feature> &feat_keyframe);

  void enhanceDepth(std::vector<Feature> &features);

  // interpolate IMU pose from clone poses, based on given IMU timestamp
  bool poseInterpolation(double timestamp, Eigen::Matrix3d &R, Eigen::Vector3d &p);

  void releaseImuBuffer(double timeline);

  void releaseDvlBuffer(double timeline);

  void releasePressureBuffer(double timeline);

  int frame_count;

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

  std::vector<std::tuple<
      double, pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string>> tuple_pcs;

  std::unordered_map<size_t, Eigen::Vector3d> truth_feature;

  // get triangulated feature position
  std::atomic<bool> is_feat;

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> trig_feat;

  std::queue<double> slide_window;
  cv::Mat img_history;
  std::recursive_mutex img_mtx;

};

} // namespace msckf_dvio


#endif // MSCKF_MANAGER_MSCKF_H

