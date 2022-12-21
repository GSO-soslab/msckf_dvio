#include "manager/msckf_manager.h"


namespace msckf_dvio
{

MsckfManager::MsckfManager(Params &parameters)
{
  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(true ? -1 : 0);
  cv::setRNGSeed(0);

  // init variable
  this->params = parameters;

  state = std::make_shared<State>(params);

  //// setup imu initializer
  switch(params.init.mode) {
    case InitMode::SETTING: {
      break;
    }

    case InitMode::STATIC: {
      break;
    }

    case InitMode::DVL_PRESSURE: {
      initializer = std::shared_ptr<InitDvlAided>(new InitDvlAided(params.init, params.prior_imu, params.prior_dvl));
      break;
    }

    default:
      break;
  }

  // initializer = std::shared_ptr<InitDvlAided>(new InitDvlAided(params.init, params.prior_imu, params.prior_dvl));
                                                      
  //// setup predictor
  predictor = std::make_shared<Predictor>(params.prior_imu);

  //// setup updater
  updater = std::make_shared<Updater>(params);

  //// setup tracker
  tracker = std::shared_ptr<TrackBase>(new TrackKLT (
    params.tracking.num_pts, params.tracking.num_aruco, params.tracking.fast_threshold,
    params.tracking.grid_x, params.tracking.grid_y, params.tracking.min_px_dist, params.tracking.pyram));

  Eigen::Matrix<double, 8, 1> cam0_calib;
  cam0_calib << params.prior_cam.intrinsics(0) * params.tracking.downsample_ratio, 
                params.prior_cam.intrinsics(1) * params.tracking.downsample_ratio, 
                params.prior_cam.intrinsics(2) * params.tracking.downsample_ratio, 
                params.prior_cam.intrinsics(3) * params.tracking.downsample_ratio,
                params.prior_cam.distortion_coeffs(0), params.prior_cam.distortion_coeffs(1), 
                params.prior_cam.distortion_coeffs(2), params.prior_cam.distortion_coeffs(3);
  if(params.tracking.cam_id == 0){
    camera_fisheye.insert({0, false});
    camera_calibration.insert({0, cam0_calib});
  }
  tracker->set_calibration(camera_calibration, camera_fisheye);

  recorder = std::make_shared<Recorder>("/home/lin/Desktop/features.txt");

  frame_count = params.keyframe.frame_count;
  frame_distance = 0;
}

void MsckfManager::feedImu(const ImuMsg &data) {
  std::unique_lock<std::mutex> lck(mtx);

  //// append to the buffer 
  buffer_imu.emplace_back(data);

  //// if imu not initialized, feed to initializer
  if(!initializer->isInit() && initializer->useSensor(Sensor::IMU)) {
    initializer->feedImu(data);
  }

  //!TODO: delete imu messages that are older then some time, 
  //!      like 10s in case 
}

void MsckfManager::feedDvl(const DvlMsg &data) {
  //! TODO: preprocess dvl:
  //!  1) correct timeoffset: basic substract time transmit from DVl to bottom, check duraction field of df21
  //!  2) remove spikes noise (Median filtering) and data smoothing (Moving Average Filter)

  std::unique_lock<std::mutex> lck(mtx);

  //// append to the buffer 
  buffer_dvl.emplace_back(data);

  // if(buffer_dvl.size()>200){
  //   printf("Manager warning: DVL velocity buffer overflow, drop now!\n");
  //   buffer_dvl.erase(buffer_dvl.begin());
  // }

  //// if imu not initialized, feed to initializer
  if(!initializer->isInit() && initializer->useSensor(Sensor::DVL)) {
    initializer->feedDvl(data);
  }
}

void MsckfManager::feedPressure(const PressureMsg &data) {
  std::unique_lock<std::mutex> lck(mtx);

  //// append to the buffer 
  buffer_pressure.emplace_back(data);

  // if(buffer_pressure.size()>600){
  //   printf("Manager warning: pressure buffer overflow, drop now!\n");
  //   buffer_pressure.erase(buffer_pressure.begin());
  // }

  //// if imu not initialized, feed to initializer
  //! TODO: check if this sensor is need for initialization
  //! initializer->needSensor(PRESSURE) ?
  if(!initializer->isInit() && initializer->useSensor(Sensor::PRESSURE)) {
    initializer->feedPressure(data);
  }
}

void MsckfManager::feedCamera(ImageMsg &data) {
  // check if system is initialized 
  // std::unique_lock<std::mutex> lck(mtx);

  //! TODO: need to store image if this not used initialization
  if(!initializer->isInit())
    return;

  // do front-end tracking for this sensor
  auto start = std::chrono::system_clock::now();
  tracker->feed_monocular(data.time, data.image, 0);
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end - start;

  // append new tracking result
  mtx.lock();

  // append sensor timestamp to buffer
  buffer_time_img.emplace(data.time);
  if(buffer_time_img.size() > 1000) {
    buffer_time_img.pop();
    printf("warning: camera time msg buffer overflow !\n");
  }

  mtx.unlock();

  // // lost features
  // std::shared_ptr<FeatureDatabase> database = tracker->get_feature_database();
  // std::vector<std::shared_ptr<Feature>> feats_lost = database->features_not_containing_newer(data.time);

  // // delete access features
  // for (auto const &f : feats_lost) {
  //   f->to_delete = true;
  // }
  // database->cleanup();

  // // marg features
  // if(buffer_time_img.size() == params.msckf.max_clone_C) {
  //   auto marg_time = buffer_time_img.front();
  //   buffer_time_img.pop();

  //   database->cleanup_measurements(marg_time);
  // }

  // updateImgHistory();
}

void MsckfManager::updateImgHistory() {
  std::lock_guard<std::recursive_mutex> lock(img_mtx);

  cv::Mat img;
  tracker->display_history(img, 0, 255, 255, 255, 255, 255);
  img_history = img.clone();
}

cv::Mat MsckfManager::getImgHistory() {
  std::lock_guard<std::recursive_mutex> lock(img_mtx);

  return img_history;
}

void MsckfManager::backend() {

/**************************************************************************************/
/****************************** Sysmtem initailization ********************************/
/**************************************************************************************/
  if(!initializer->isInit()) {

    initializer->checkInit();

    if(!initializer->isInit()) {
      return;
    }

    // update to initialized result
    std::map<Sensor, double> timelines;
    initializer->updateInit(state, params, timelines);

    // clean those used in initialization in global buffer
    for (const auto& kv : timelines) {
      switch(kv.first) {
        case Sensor::IMU: {
          releaseImuBuffer(kv.second);
          break;
        }

        case Sensor::DVL: {
          releaseDvlBuffer(kv.second);
          break;
        }

        case Sensor::PRESSURE: {
          releasePressureBuffer(kv.second);
          break;
        }

        default:
          break;
      }
    }

  }

/******************** Check Initialization with given state ********************/
  // if(!imu_initializer->isInitialized()) {
    
  //   imu_initializer->checkInitGiven();

  //   if(imu_initializer->isInitialized()){

  //     //// system initialized, get init result
  //     Eigen::Matrix<double, 17, 1> state_imu;
  //     Eigen::Matrix<double, 2, 1>  state_dvl;

  //     std::tie(state_imu, state_dvl) = imu_initializer->getInitResult();

  //     //// update IMU state
  //     state->setTimestamp(state_imu(0));
  //     state->setImuValue(state_imu.tail(16));

  //     //// update DVL state
  //     if(params.msckf.do_time_I_D)
  //       state->setEstimationValue(DVL, EST_TIMEOFFSET, Eigen::MatrixXd::Constant(1,1,state_dvl(1)));
  //   }
  //   else
  //     //// system not initialized, return 
  //     return;
  // }


/**************************************************************************************/
/***************************** Update for multi-sensors *******************************/
/**************************************************************************************/

  //! TODO: check efficiency on "swith" or "if-else"

  switch(selectUpdateSensor()) {

    // choose DVL BT velocity to update IMU
    case DVL: {

      doDVL();

      // // individual BT veloicty update 
      // doDvlBT();
      break;
    }

    // choose DVL CP pressure to update IMU
    case PRESSURE: {

      doDVL();

      // // individual pressure update
      // doPressure();
      break;
    }

    // choose Camera to update IMU
    case CAM0: {

      // doCameraSlideWindow();

      doCameraKeyframe();

      break;
    }

    // no sensor data avaiable
    case NONE:  {
      break;
    }
  }
}

Sensor MsckfManager::selectUpdateSensor() {
  double early_time = std::numeric_limits<double>::infinity();
  Sensor update_sensor = NONE;

  mtx.lock();

  // check Camera buffer
  if(buffer_time_img.size() > 0){
    // get first sensor timestamp into IMU frame
    double time = buffer_time_img.front();
    if(params.msckf.do_time_C_I)
      time += state->getEstimationValue(CAM0, EST_TIMEOFFSET)(0);
    else
      time += params.prior_cam.timeoffset;

    // compare with other sensor
    if(time < early_time){
      early_time = time;
      update_sensor = CAM0;
    }
  }

  //! NOTE: make sure do pressure first, we will update pressure with velocity,
  //!       if velocity and pressure are same timestamp, then velocity no need to update 

  //// check DVL pressure buffer
  if(buffer_pressure.size() > 0){
    // get first sensor timestamp into IMU frame
    double time = buffer_pressure.front().time;
    if(params.msckf.do_time_I_D)
      time += state->getEstimationValue(DVL, EST_TIMEOFFSET)(0);
    else
      time += params.prior_dvl.timeoffset;

    // compare with other sensor
    if(time < early_time){
      early_time = time;
      update_sensor = PRESSURE;
    }
  }

  //// check DVL velocity buffer
  if(buffer_dvl.size() > 0) {
    // get first sensor timestamp into IMU frame
    double time = buffer_dvl.front().time;
    if(params.msckf.do_time_I_D)
      time += state->getEstimationValue(DVL, EST_TIMEOFFSET)(0);
    else
      time += params.prior_dvl.timeoffset;

    // compare with other sensor
    if(time < early_time){
      early_time = time;
      update_sensor = DVL;
    }
  }

  // //! TEST:
  // if(update_sensor == CAM0) {
  //   printf("Img time: %.9f\n", early_time);
  // }
  // if(update_sensor == PRESSURE) {
  //   printf("pre time: %.9f\n", early_time);
  // }
  // if(update_sensor == DVL) {
  //   printf("vel time: %.9f\n", early_time);
  // }

  mtx.unlock();

  return update_sensor;
}

void MsckfManager::doCameraKeyframe() {

  // ------------------------------  Select Data ------------------------------ // 

  std::vector<ImuMsg> selected_imu;
  bool do_update = true;

  mtx.lock();

  // [0] select IMU time duration: 
  // convert sensor time into IMU time
  auto time_offset = params.msckf.do_time_C_I ? 
                     state->getEstimationValue(CAM0, EST_TIMEOFFSET)(0) : 
                     params.prior_cam.timeoffset;
  auto time_curr_sensor = buffer_time_img.front();
  auto time_prev_state = state->getTimestamp();
  auto time_curr_state = time_curr_sensor + time_offset;
  // make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    // printf("Manger warning: new image time:%f "
    //        "is eariler then current state time:%f, drop it now!\n",
    //        time_curr_state, time_prev_state);

    // erase this image data because bad timestamp
    buffer_time_img.pop();
    do_update = false;
  }
  mtx.unlock();

  if(!do_update) {
    return;
  }

  mtx.lock();

  // make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    // [1] select IMU
    selected_imu = selectImu(time_prev_state, time_curr_state);
    
    // [2] clean buffer
    // earse selected IMU data, leave one for interpolation
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);  
    // erase selected image data
    buffer_time_img.pop();
  }

  mtx.unlock();

  // if no enough IMU data, just return
  if(selected_imu.size()<1) {
    return;
  }

  // ------------------------------  Do Update ------------------------------ // 

  //// [0] IMU Propagation
  predictor->propagate(state, selected_imu);
  assert(!state->foundSPD("cam_propagate"));

  //// [1] State Augmentation: 
  Eigen::Vector3d w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);

  // check if relative motion is larger enough to insert a clone
  // get last clone pose, also meaning key-frame during feature tracking

  bool insert_clone = checkMotion() && checkScene(time_curr_sensor);

  // use Last angular velocity for cloning when estimating time offset)
  if(params.msckf.max_clone_C > 0 && insert_clone) {
    // clone IMU pose, augment covariance
    predictor->augment(CAM0, CLONE_CAM0, state, time_curr_sensor, w_I);

    printf("[TEST]: new clone:%d\n", state->getEstimationNum(CLONE_CAM0));
  }

  // [2] select tracked features
  std::vector<Feature> feature_lost;
  std::vector<Feature> feature_marg;
  // selectFeaturesSlideWindow(time_curr_sensor, feature_lost, feature_marg);
  selectFeaturesKeyFrame(time_curr_sensor, feature_lost, feature_marg);

  // [3] Camera Feature Update: feature triangulation, feature update 

  // feature triangulation
  std::vector<Feature> feature_msckf;
  updater->cameraMeasurementKeyFrame(state, feature_lost, feature_marg, feature_msckf);

  //! TEST: check feature update analysis 
  // if(feature_msckf.size()>0) {
  //   auto add = 0;
  //   for(const auto& f : feature_msckf) {
  //     add += f.timestamps.at(0).size();
  //   }

  //   file.open(file_path, std::ios_base::app);//std::ios_base::app
  //   file<<std::setprecision(17)<<time_curr_sensor<<","<<add<<std::endl;
  //   file.close();
  // }

  //! TEST: manual set the depth of features
  // for(auto& feat : feature_msckf) {
  //   feat.p_FinG(2) = 4.2;
  // }

  // update triangulation to the database
  tracker->get_feature_database()->update_new_triangulation(feature_msckf);

  // do the camera update
  updater->updateCam(state, feature_msckf);

  // setup new MSCKF features for visualization
  setFeatures(feature_msckf);

  //// [4] Marginalization(if reach max clone): 
  if((params.msckf.max_clone_C > 0) &&
     (params.msckf.max_clone_C == state->getEstimationNum(CLONE_CAM0))) {

    // Marginalization

    // remove marginalized feature measurements, lost feature measurements alrady remove when we select
    tracker->get_feature_database()->cleanup_marg_measurements(feature_marg);

    // remove oldest clone state and covaraince
    auto marg_index_0 = params.msckf.marginalized_clone.at(0);
    updater->marginalize(state, CLONE_CAM0, marg_index_0);

    // remove feature measurements only older then oldest clone timestamp 
    auto oldest_time = state->getCloneTime(CLONE_CAM0, 0);
    tracker->get_feature_database()->cleanup_out_measurements(oldest_time);

    printf("[TEST]: aft clean:%d\n", state->getEstimationNum(CLONE_CAM0));
  }

}

void MsckfManager::doCameraSlideWindow() {

  // ------------------------------  Select Data ------------------------------ // 

  std::vector<ImuMsg> selected_imu;
  bool do_update = true;

  mtx.lock();

  // [0] select IMU time duration: 
  // convert sensor time into IMU time
  auto time_offset = params.msckf.do_time_C_I ? 
                     state->getEstimationValue(CAM0, EST_TIMEOFFSET)(0) : 
                     params.prior_cam.timeoffset;
  auto time_curr_sensor = buffer_time_img.front();
  auto time_prev_state = state->getTimestamp();
  auto time_curr_state = time_curr_sensor + time_offset;
  // make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger warning: new image time:%f "
           "is eariler then current state time:%f, drop it now!\n",
           time_curr_state, time_prev_state);

    // erase this image data because bad timestamp
    buffer_time_img.pop();
    do_update = false;
  }
  mtx.unlock();

  if(!do_update) {
    return;
  }

  mtx.lock();

  // make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    // [1] select IMU
    selected_imu = selectImu(time_prev_state, time_curr_state);
    
    // [2] clean buffer
    // earse selected IMU data, leave one for interpolation
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);  
    // erase selected image data
    buffer_time_img.pop();
  }

  mtx.unlock();

  // if no enough IMU data, just return
  if(selected_imu.size()<1) {
    return;
  }

  // ------------------------------  Do Update ------------------------------ // 

  // printf("do cam\n");
  // printf("    IMU:%ld\n", selected_imu.size());

  //// [0] IMU Propagation
  predictor->propagate(state, selected_imu);
  assert(!state->foundSPD("cam_propagate"));

  //// [1] State Augmentation: 
  Eigen::Vector3d w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);

  // use Last angular velocity for cloning when estimating time offset)
  if(params.msckf.max_clone_C > 0) {
    // clone IMU pose, augment covariance
    predictor->augment(CAM0, CLONE_CAM0, state, time_curr_sensor, w_I);
  }

  // [2] select tracked features
  std::vector<Feature> feature_MSCKF;
  selectFeatures(time_curr_sensor,feature_MSCKF);

  // [3] Camera Feature Update: feature triangulation, feature update 
  updater->cameraMeasurement(state, feature_MSCKF);
  updater->updateCam(state, feature_MSCKF);
  setFeatures(feature_MSCKF);

  //// [4] Marginalization(if reach max clone): 
  if((params.msckf.max_clone_C > 0) &&
     (params.msckf.max_clone_C == state->getEstimationNum(CLONE_CAM0))) {
    // Cleanup any features older then the marginalization time
    //! TODO: this clean will make memory issue
    tracker->get_feature_database()->cleanup_measurements(state->getMarginalizedTime(CLONE_CAM0));
    
    // remove the clone and related covarinace
    int index = 0;
    updater->marginalize(state, CLONE_CAM0, index);
  }

}

bool MsckfManager::checkScene(double curr_time) {
  // [0] Check current frame feature

  // find feature measurements at current frame timestamp
  std::vector<Feature> curr_frame_feat;
  tracker->get_feature_database()->features_measurement_selected(curr_time, curr_frame_feat, true);

  // only the detected feature reach the minimum requirement 
  if(curr_frame_feat.size() < params.keyframe.min_tracked) {
    return false;
  }

  // [1] Get last keyframe feature (associated with last clone pose)

  // if no clone exist, then insert current one
  auto clone_size = state->getEstimationNum(CLONE_CAM0);
  if(clone_size == 0) {
    return true;
  }

  // get clone timestamp
  auto clone_time = state->getCloneTime(CLONE_CAM0, clone_size-1);

  // find feature measurements at last keyframe timestamp
  std::vector<Feature> key_frame_feat;
  tracker->get_feature_database()->features_measurement_selected(clone_time, key_frame_feat, true);

  // [2] Compare the scene

  // find features numbers that tracked from last keyframe
  int tracked_count = 0;
  for(const auto& key_feat : key_frame_feat) {
    for(const auto& curr_feat : curr_frame_feat) {
      if(key_feat.featid == curr_feat.featid) {
        tracked_count ++;
        break;
      }
    }
  }

  // compare with parameters ratio
  double ratio = tracked_count / curr_frame_feat.size();

  if(ratio < params.keyframe.scene_ratio) {
    return true;
  }
  else {
    return false;
  }
}

bool MsckfManager::checkMotion() {
  // if no clone exist, then insert current one
  auto clone_size = state->getEstimationNum(CLONE_CAM0);
  if(clone_size == 0) {
    return true;
  }

  // get clone pose 
  Eigen::VectorXd pose = state->getClonePose(CLONE_CAM0, clone_size-1);   
  Eigen::Matrix3d R_Ii_G = toRotationMatrix(pose.block(0,0,4,1));
  Eigen::Vector3d p_G_Ii = pose.block(4,0,3,1);

  // get current IMU pose
  Eigen::Vector3d p_G_Ij = state->getEstimationValue(IMU, EST_POSITION);

  // caculate distance between latest clone and current pose
  //! TODO: better method to determine pose constraint ?
  //! TODO: two-way marginalization from S. Shen et. al. 2014 ISER Initialization-free VIO

  double distance = 0;
  if(params.keyframe.motion_space == 2) {
    // the disance from Ij to Ii at global frame
    // p_Ii_Ij = p_G_Ij - p_G_Ii 
    Eigen::Vector3d p_Ii_Ij = p_G_Ij - p_G_Ii;

    distance = sqrt(p_Ii_Ij(0)*p_Ii_Ij(0) + p_Ii_Ij(1)*p_Ii_Ij(1));
  }
  else if (params.keyframe.motion_space == 3) {
    // the disance from Ij to Ii at Ii frame
    // p_Ii_Ij = R_G_Ii^T (p_G_Ij - p_G_Ii)
    Eigen::Vector3d p_Ii_Ij = R_Ii_G * (p_G_Ij - p_G_Ii);

    distance = sqrt(p_Ii_Ij(0)*p_Ii_Ij(0) + p_Ii_Ij(1)*p_Ii_Ij(1) + p_Ii_Ij(2)*p_Ii_Ij(2));
  }

  if(distance >= params.keyframe.frame_motion) {
    return true;
  }
  else {
    return false;
  }
}

bool MsckfManager::checkFrameCount() {
  // update count of image frames
  frame_count ++;

  if(frame_count >= params.keyframe.frame_count) {
    frame_count = 0;
    return true;
  }
  else {
    return false;
  }
}                  
        

void MsckfManager::releaseImuBuffer(double timeline) {
  std::unique_lock<std::mutex> lck(mtx);

  auto frame = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time >= timeline;});

  if (frame != buffer_imu.end()){
    buffer_imu.erase(buffer_imu.begin(), frame);
  }
}

void MsckfManager::releaseDvlBuffer(double timeline) {
  std::unique_lock<std::mutex> lck(mtx);

  auto frame = std::find_if(buffer_dvl.begin(), buffer_dvl.end(),
                [&](const auto& dvl){return dvl.time >= timeline;});

  if (frame != buffer_dvl.end()){
    last_dvl = *(frame);
    buffer_dvl.erase(buffer_dvl.begin(), frame);
  }     

}

void MsckfManager::releasePressureBuffer(double timeline) {
  std::unique_lock<std::mutex> lck(mtx);

  auto frame = std::find_if(buffer_pressure.begin(), buffer_pressure.end(),
                [&](const auto& pressure){return pressure.time >= timeline;});

  if (frame != buffer_pressure.end()) {
    buffer_pressure.erase(buffer_pressure.begin(), frame);
  }  
}

void MsckfManager::setFeatures(std::vector<Feature> &features) {
  std::unique_lock<std::mutex> lck(mtx);

  // get feature
  for (size_t f = 0; f < features.size(); f++) {
    trig_feat.emplace_back(features[f].p_FinG);
  }
}

std::vector<Eigen::Vector3d> MsckfManager::getFeatures() {
  std::unique_lock<std::mutex> lck(mtx);

  std::vector<Eigen::Vector3d> out_trig_feat;

  // copy triangulated message
  std::copy(trig_feat.begin(), trig_feat.end(), std::back_inserter(out_trig_feat)); 

  // clean
  std::vector<Eigen::Vector3d>().swap(trig_feat);

  return out_trig_feat;
}


void MsckfManager::doDvlBT() {

  // ------------------------------  Select Data ------------------------------ // 

  std::vector<ImuMsg> selected_imu;
  DvlMsg selected_dvl;

  mtx.lock();

  // [0] select DVL
  selected_dvl = buffer_dvl.front();

  // [1] select IMU time duration: 
  // convert sensor time into IMU time
  auto time_offset = params.msckf.do_time_I_D ? 
                     state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
                     params.prior_dvl.timeoffset;
  auto time_curr_sensor = selected_dvl.time;
  auto time_prev_state = state->getTimestamp();
  auto time_curr_state = time_curr_sensor + time_offset;
  // make sure sensor data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: DVL BT time:%f is eariler then current state time:%f, drop it now!\n",
            time_curr_state, time_prev_state);
    std::exit(EXIT_FAILURE);
  }

  //// make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    // [2] select IMU
    selected_imu = selectImu(time_prev_state, time_curr_state);

    // [3] clean buffer
    // erase selected IMU, but leave one for next interpolate
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);  
    // erase selected dvl
    buffer_dvl.erase(buffer_dvl.begin());
  }

  mtx.unlock();

  // ------------------------------  Do update ------------------------------ // 

  // if no enough IMU data, just return
  if(selected_imu.size()<1) {
    return;
  }
  printf("\nDVL:%ld\n", selected_imu.size());

  // [0] IMU Propagation
  predictor->propagate(state, selected_imu);
  assert(!state->foundSPD("dvl_bt_propagate"));

  // [1] State Augment: clone IMU pose, augment covariance
  Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
  Eigen::Vector3d last_v_D = selected_dvl.v;
  // use Last angular velocity for cloning when estimating time offset)
  if(params.msckf.max_clone_D > 0)
    predictor->augmentDvl(state, time_curr_sensor, last_w_I);

  // [2] State Update
  updater->updateDvl(state, last_w_I, last_v_D);
  // updater->updateDvlSimple(state, last_w_I, last_v_D, true);

  // [3] Marginalization: if reach the max clones, remove the clone and related covarinace
  if( (params.msckf.max_clone_D > 0) &&
      (params.msckf.max_clone_D < state->getEstimationNum(CLONE_DVL)) ) {
    int index = 0;
    updater->marginalize(state, CLONE_DVL, index);
  }

}

/** 
 * @brief do the pressure data update for states  
 *
 * @details 1) interpolate velocity at pressure time; 
 *          2) do both veloicty and position update; 
 * 
 * @todo add pressure as individual sensor setup, e.g. prior information, not from DVL
 */
void MsckfManager::doPressure() {

  // ------------------------------  Select Data ------------------------------ // 

  std::vector<ImuMsg> selected_imu;
  PressureMsg selected_pres;
  DvlMsg selected_dvl;

  getDataForPressure(selected_pres, selected_dvl, selected_imu);

  // mtx.lock();

  // // make sure has new DVL velocity after Pressure data 
  // // because pressure only update with velocity or interpolated velocity
  // if((buffer_dvl.size() > 0) && 
  //    (buffer_pressure.front().time <= buffer_dvl.front().time)) {

  //   // [0] select pressure 
  //   selected_pres = buffer_pressure.front();

  //   // [3] select DVL
  //   printf("last dvl t:%f\n",last_dvl.time);
  //   printf("dvl size:%ld, front time:%f\n", buffer_dvl.size(), buffer_dvl.front().time);
  //   printf("pressure t: %f\n",selected_pres.time);

  //   selected_dvl = interpolateDvl(last_dvl, buffer_dvl.front(), selected_pres.time);

  //   // [1] select IMU time duration
  //   // convert sensor time into IMU time
  //   auto offset = params.msckf.do_time_I_D ? 
  //                 state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
  //                 params.prior_dvl.timeoffset;
  //   double time_prev_state = state->getTimestamp();
  //   double time_curr_state = selected_pres.time + offset;
  //   // make sure sensor data is not eariler then current state
  //   if(time_curr_state <= time_prev_state) {
  //     printf("Manger error: new pressure measurement time:%f" 
  //            "is eariler then current state time:%f\n",
  //            time_curr_state, time_prev_state);
  //     std::exit(EXIT_FAILURE);
  //   }

  //   // make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  //   auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
  //                 [&](const auto& imu){return imu.time > time_curr_state ;});
  //   if(frame_imu != buffer_imu.end()) {
  //     // [2] select IMU
  //     selected_imu = selectImu(time_prev_state, time_curr_state);

  //     // [4] clean buffer
  //     // erase selected pressure
  //     buffer_pressure.erase(buffer_pressure.begin());
  //     // erase selected IMU, but leave one for next interpolate
  //     buffer_imu.erase(buffer_imu.begin(), frame_imu-1);   
  //     // store for next interpolate, 
  //     last_dvl = buffer_dvl.front(); 
  //     // only erase selected DVL if velocity and pressure at same timestamp(e.g. both from DVL BT)
  //     if(buffer_pressure.front().time == buffer_dvl.front().time) {
  //       buffer_dvl.erase(buffer_dvl.begin());  
  //     }
  //   }
  // }

  // mtx.unlock();

  // ------------------------------  Update ------------------------------ // 

  // if no enough IMU data, just return
  if(selected_imu.size()<1) {
    return;
  }
  printf("\nPressure:%ld\n", selected_imu.size());

  // [0] IMU propagation
  predictor->propagate(state, selected_imu);
  assert(!state->foundSPD("pressure_propagate"));

  // auto imu_value = getNewImuState();
  // printf("imu0:q=%f,%f,%f,%f,p:%f,%f,%f\n",imu_value(0),imu_value(1),imu_value(2),imu_value(3)
  //         ,imu_value(4),imu_value(5),imu_value(6));

  // [1] State Update with velocity(interpolated or same time)
  Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
  Eigen::Vector3d last_v_D = selected_dvl.v;
  updater->updateDvl(state, last_w_I, last_v_D);
  // updater->updateDvlSimple(state, last_w_I, last_v_D, true);

  // std::cout<<"v_D: "<< last_v_D.transpose()<<std::endl;
  // imu_value = getNewImuState();
  // printf("imu1:q=%f,%f,%f,%f,p:%f,%f,%f\n",imu_value(0),imu_value(1),imu_value(2),imu_value(3)
  //         ,imu_value(4),imu_value(5),imu_value(6));

  // [2] State Update with Pressure
  // get the begin pressure value
  double pres_init = state->getPressureInit();
  double pres_curr = selected_pres.p;
  // update
  updater->updatePressure(state, pres_init, pres_curr, true);

}

//! TODO: DVL/pressure can still update without IMU propagation
//!       check how many IMU inside DVL/pressure update

void MsckfManager::doDVL() {

/**************************************************/
/**************************************************/
/**************************************************/

  //// if pressure earlier then velocity
    // data: interpolated velocity, pressure
    // clean: erase pressure buffer

  //// if pressure same as velocity
    // data: velocity, pressure
    // clean: erase velocity, pressure buffer

  //// if no pressure
    // data: velocity
    // clean: erase velocity buffer

  //// update:
    // velocity update
       // select IMU based on velocity time
       // IMU propagation
       // do update
    // pressure update
      // do update

  /******************** grab DVL BT velocity and pressure ********************/

  bool do_pressure = false;
  bool do_velocity = false;
  bool erase_pressure = false;
  bool erase_velocity = false;
  DvlMsg new_dvl;
  PressureMsg new_pres;

  mtx.lock();

  if(buffer_pressure.size() > 0 && 
    //  buffer_pressure.front().time < buffer_dvl.front().time &&
     buffer_pressure.front().time <= state->getTimestamp()){
    //// this pressure timetsamp is wrong: maybe dely by ros callback, should be process by previous velocity

    printf("Manager warning: pressure time:%f is before state time:%f\n, drop it now!", 
            buffer_pressure.front().time, state->getTimestamp());
    buffer_pressure.erase(buffer_pressure.begin());
  }
  else if(buffer_pressure.size() > 0 && buffer_dvl.size() > 0 &&
          buffer_pressure.front().time < buffer_dvl.front().time){
    // printf("do intepolated velocity-pressure \n");

    //// pressure earlier then velocity: 
    ////    pressure not measure same time as velocity(CP-pressure, or individual pressure sensor)

    // get data
    new_pres = buffer_pressure.front();
    new_dvl = interpolateDvl(last_dvl, buffer_dvl.front(), new_pres.time);
    do_pressure = true;
    do_velocity = true;

    // clean
    erase_pressure = true;
  }
  else if(buffer_pressure.size() > 0 && buffer_dvl.size() > 0 &&
          buffer_pressure.front().time == buffer_dvl.front().time){
    // printf("do same velocity-pressure \n");

    //// pressure same as velocity: 
    ////    pressure measure same time as velocity(CP-pressure and CP-velocity)

    // get data
    new_pres = buffer_pressure.front();
    new_dvl = buffer_dvl.front();
    do_pressure = true;
    do_velocity = true;


    // clean buffer
    last_dvl = buffer_dvl.front(); 
    erase_pressure = true;
    erase_velocity = true;
  }
  else if(
          // buffer_pressure.size() == 0 ||
          // buffer_pressure.front().time > buffer_dvl.front().time)
          buffer_dvl.size() > 0)  {
    // printf("do pure velocty\n");

    //// pure DVL update

    // get data
    new_dvl = buffer_dvl.front();
    do_velocity = true;


    // clean buffer
    last_dvl = buffer_dvl.front(); 
    erase_velocity = true;
  }
  // else{
  //   printf("\n---------------\nDVL-BT-Velocity:\n");
  //   if(buffer_dvl.size()>0){
  //     for(const auto &data: buffer_dvl)
  //       printf("t: %f, ", data.time);
  //   }

  //   printf("\nDVL-CP-Pressure:\n");
  //   if(buffer_pressure.size()>0){
  //     for(const auto &data: buffer_pressure)
  //       printf("t: %f, ", data.time);
  //   }
  // }


  //! TEST: for only IMU-Velocity MSCKF
  // new_dvl = buffer_dvl.front();
  // do_velocity = true;

  // printf("do IMU propagation + velocty update\n");

  // // clean buffer
  // last_dvl = buffer_dvl.front(); 
  // erase_velocity = true;

  mtx.unlock();


  if(!do_velocity){
    return;
  }

/******************** grab IMU data for propagation ********************/

  std::vector<ImuMsg> selected_imu;

  //// determine selected IMU range, from last updated to current sensor measurement
  auto time_offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : params.prior_dvl.timeoffset;
  auto time_curr_sensor = new_dvl.time;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = time_curr_sensor + time_offset;

  //// make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new velocity time:%f is eariler then current state time:%f, drop it now!\n",
            time_curr_state, time_prev_state);

    mtx.lock();
    // erase selected pressure data
    if(erase_pressure){
      buffer_pressure.erase(buffer_pressure.begin());
    }
    // erase selected DVL BT velocity data
    if(erase_velocity){
      buffer_dvl.erase(buffer_dvl.begin());
    }
    mtx.unlock();

    return;
  }

  mtx.lock();

  //// make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    //// make sure we have at least 2 IMU data
    selected_imu = selectImu(time_prev_state, time_curr_state);

    // earse selected IMU data
    // leave one IMU data time <= current state time, so we can interploate later
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);  
    // erase selected pressure data
    if(erase_pressure){
      buffer_pressure.erase(buffer_pressure.begin());
    }
    // erase selected DVL BT velocity data
    if(erase_velocity){
      buffer_dvl.erase(buffer_dvl.begin());
    }
    
  }
  // else{
  //   printf("Manger warning: current IMU time:%f not > current sensor time:%f\n", buffer_imu.back().time, time_curr_state);
  // }

  mtx.unlock();

  /******************** imu propagation + velocity update + pressure update(if)********************/

  if(selected_imu.size()>0){
    // printf("    IMU:%ld\n\n", selected_imu.size());
    // IMU propagation
    predictor->propagate(state, selected_imu);
    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    assert(!state->foundSPD("dvl_propagate"));

    if(do_velocity){

      // Last angular velocity and linear velocity
      Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
      Eigen::Vector3d last_v_D = new_dvl.v;

      //// use Last angular velocity for cloning when estimating time offset)
      if(params.msckf.max_clone_D > 0)
        predictor->augmentDvl(state, time_curr_sensor, last_w_I);

      // update
      updater->updateDvl(state, last_w_I, last_v_D);
      // updater->updateDvlSimple(state, last_w_I, last_v_D, true);

      // marginalize 
      // if max clone of DVL is reached, then do the marginalize: remove the clone and related covarinace
      if( (params.msckf.max_clone_D > 0) &&
          (params.msckf.max_clone_D < state->getEstimationNum(CLONE_DVL)) ) {
        int index = 0;  
        updater->marginalize(state, CLONE_DVL, index);
      }
    }

    if(do_pressure){
      // get the begin pressure value
      double pres_init = state->getPressureInit();
      double pres_curr = new_pres.p;

      // update
      // updater->updatePressureSimple(state, pres_init, pres_curr);
      updater->updatePressure(state, pres_init, pres_curr, true);
    }

    // //! TEST: update velocity and pressure at same time
    // if(do_velocity && do_pressure) {
    //   // Last angular velocity and linear velocity
    //   Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
    //   Eigen::Vector3d last_v_D = new_dvl.v;

    //   // get the begin pressure value
    //   double pres_init = state->getPressureInit();
    //   double pres_curr = new_pres.p;

    //   updater->updateDvlPressureSimple(state,last_w_I, last_v_D, pres_init, pres_curr);
    //   // updater->updateDvlPressure(state,last_w_I, last_v_D, pres_init, pres_curr);
    // }
    // else if (do_velocity) {
    //   // Last angular velocity and linear velocity
    //   Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
    //   Eigen::Vector3d last_v_D = new_dvl.v;

    //   // update
    //   // updater->updateDvl(state, last_w_I, last_v_D);
    //   updater->updateDvlSimple(state, last_w_I, last_v_D, true);
    // }
    
  }

}

void MsckfManager::doPressure_test() {

  /********************* no imu propagation ********************/
  // // select pressure data
  // PressureMsg selected_pres = buffer_pressure.front();
  // buffer_pressure.erase(buffer_pressure.begin());

  // // get the begin pressure value
  // double pres_init = state->getPressureInit();
  // double pres_curr = selected_pres.p;

  // // update
  // updater->updatePressure(state, pres_init, pres_curr, true);


  /******************** IMU propagation, pressure update ********************/

  //! TODO: deal with pressure from BT, no need propagation
  //// if state time = pressure time: (pressure from BT)
  ////    means pressure data from DVL BT, IMU alredy propagated in DVL update 
  //// if state time < pressure time: (pressure from CP)
  ////    means pressure data from individual pressure sensing(individual pressure sensor or pressure from CP) 

  PressureMsg selected_pres;
  std::vector<ImuMsg> selected_imu;

  mtx.lock();

  //// select first pressure data
  selected_pres = buffer_pressure.front();

  //// deal with pressure from CP
  //// determine selected IMU range, from last updated to current sensor measurement
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
                params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = selected_pres.time + offset;

  //// make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new pressure measurement time:%f is eariler then current state time:%f\n",
            time_curr_state, time_prev_state);
    // std::exit(EXIT_FAILURE);
  }

  //// make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    //// make sure we have at least 2 IMU data
    selected_imu = selectImu(time_prev_state, time_curr_state);

    // earse selected sensor measurement
    buffer_pressure.erase(buffer_pressure.begin());
    // earse selected IMU data
    // leave one IMU data time <= current state time, so we can interploate later
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);      
  }

  mtx.unlock();


  /******************** do pressure msckf update ********************/

  if(selected_imu.size() > 0){
    printf("do IMU propagtion + pressure update\n");

    predictor->propagate(state, selected_imu);

    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    assert(!state->foundSPD("press_test_propagate"));

    // get the begin pressure value
    double pres_init = state->getPressureInit();
    double pres_curr = selected_pres.p;

    // update
    updater->updatePressure(state, pres_init, pres_curr, true);

  }


}

std::vector<ImuMsg> MsckfManager::selectImu(double t_begin, double t_end) {

  std::vector<ImuMsg> selected_data;

  // BEGINNING:
  //// interpolate the beginning IMU data with two IMU data that eariler and later then this IMU
  //// if beginning IMU time just equal to existing data, interpolate with itself and later. result is itself
  auto frame = std::find_if(buffer_imu.begin(), buffer_imu.end(),
               [&](const auto& imu){return imu.time > t_begin ;});

  if(frame != buffer_imu.end() && frame != buffer_imu.begin()) {
    auto interpolated_begin = msckf_dvio::interpolateImu(*(frame-1), *frame, t_begin);
    selected_data.emplace_back(interpolated_begin);
  }

  // MIDDLE:
  //// copy IMU data inside beginning and end time range
  for(; frame != buffer_imu.end(); frame++) {
    //// break if IMU data later then or equal to selection end
    //// if equal to the end, this should be select as end method next section
    if(frame->time >= t_end)
      break;

    selected_data.emplace_back(*frame);
  }

  // END:
  //// interpolate the end IMU data with two IMU data that eariler and later then this IMU
  //// if end IMU time just equal to existing data, interpolate with eariler and itself. result is itself
  if(frame != buffer_imu.end()) {
    auto interpolated_end = msckf_dvio::interpolateImu(*(frame-1), *frame, t_end);
    selected_data.emplace_back(interpolated_end);
  }

  if(selected_data.empty())
    printf("Manger: no IMU selected for progagation\n");
  else if (selected_data.size() < 2)
    printf("Manger: only 1 IMU selected, sensor synchronization may messed up\n");

  assert(selected_data.size() > 1);

  return selected_data;
}

void MsckfManager::selectFeaturesKeyFrame(
    const double time_update, std::vector<Feature> &feat_lost, std::vector<Feature> &feat_marg) {
  // ------------------------------- Grab features ------------------------------------------ //

  // Grab lost features:
  //    1) grab whole the measuremens for each lost feature
  //    2) delete the grabbed features in the database
  tracker->get_feature_database()->features_lost(time_update, feat_lost, true, true);

  // Grab maginalized features
  //    1) select the oldest clone time
  //    2) grab whole the measurements for each feature that contain the given timestamp
  //    3) not delete
  if(state->getEstimationNum(CLONE_CAM0) == params.msckf.max_clone_C) {
    // Grab marg feature 0 
    // get oldest clone time
    auto index = params.msckf.marginalized_clone.at(0);
    auto time_oldest = state->getCloneTime(CLONE_CAM0, index);
    // get feature contain this marg time
    tracker->get_feature_database()->features_selected(time_oldest, feat_marg, false, true);

    // // Grab marg feature 1
    // // get second latest clone time
    // index = params.msckf.marginalized_clone.at(1);
    // auto time_second_latest = state->getCloneTime(CLONE_CAM0, index);
    // // get feature contain this marg time
    // std::vector<Feature> feat_marg_1;
    // tracker->get_feature_database()->features_selected(time_second_latest, feat_marg_1, false, true);

    // // Combine both marg features
    // // get marg feature 0
    // feat_marg = feat_marg_0;
    // // get marg feature 1
    // for(const auto& feat_1 : feat_marg_1) {
    //   // check if feat_1 exist in feat_0 list
    //   auto exist = std::find_if(feat_marg_0.begin(), feat_marg_0.end(),
    //               [&](const auto& feat_0){return feat_0.featid == feat_1.featid ;});
    //   // if not exist, then add it
    //   if(exist == feat_marg_0.end()) {
    //     feat_marg.emplace_back(feat_1);
    //   }
    // }

  }

}


/**
 *  @brief get all the data need for pressure update, e.g. pressure, DVL BT velocity, IMU
 * 
 *  @param pressure: selected pressure data for this update
 *  @param dvl: selected DVL velocity for this update, same time or interpolated
 *  @param imus: selected a series of IMU for propagation
 * 
 */
void MsckfManager::getDataForPressure(PressureMsg &pressure, DvlMsg &dvl, std::vector<ImuMsg> &imus) {
  std::unique_lock<std::mutex> lck(mtx);

  // [0] check if DVL is avaiable
  if(buffer_dvl.size() < 0 || 
     buffer_pressure.front().time > buffer_dvl.front().time){
    return;
  }

  // [1] select pressure 
  pressure = buffer_pressure.front();

  // [2] select DVL: pressure inside last and next DVL
  if(pressure.time > last_dvl.time && 
     (pressure.time < buffer_dvl.front().time || 
      pressure.time == buffer_dvl.front().time) ) {
    dvl = interpolateDvl(last_dvl, buffer_dvl.front(), pressure.time);
  }
  else {
    // send warning
    printf("Manager warning: pressure interpolated failed, "
           "last t:%f, interpoated t:%f, next t:%f, will drop this pressure\n",
           last_dvl.time, pressure.time, buffer_dvl.front().time);
    // delete bad pressure
    buffer_pressure.erase(buffer_pressure.begin());

    return;
  }

  // [3] select IMU time duration
  // convert sensor time into IMU time
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
                params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = pressure.time + offset;
  // make sure sensor data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new pressure measurement time:%f" 
            "is eariler then current state time:%f\n",
            time_curr_state, time_prev_state);
    std::exit(EXIT_FAILURE);
  }

  // [4] select IMU data: make sure IMU time > current sensor time for interpolation
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                   [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    imus = selectImu(time_prev_state, time_curr_state);
  }

  // [5] clean buffer
  if(imus.size()>1) {
    // erase selected pressure
    buffer_pressure.erase(buffer_pressure.begin());

    // store for next interpolate, 
    last_dvl = buffer_dvl.front(); 
    // only erase selected DVL if velocity and pressure at same timestamp(e.g. both from DVL BT)
    if(buffer_pressure.front().time == buffer_dvl.front().time) {
      buffer_dvl.erase(buffer_dvl.begin());  
    }

    // erase selected IMU, but leave one for next interpolate
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);   
  }

}

void MsckfManager::selectFeaturesSlideWindow(
    const double time_update, std::vector<Feature> &feat_lost, std::vector<Feature> &feat_marg) {

  // ------------------------------- Grab features ------------------------------------------ //

  // Grab lost features:
  //    1) grab whole the measuremens for each lost feature
  //    2) delete the grabbed features in the database
  tracker->get_feature_database()->features_lost(time_update, feat_lost, true, true);

  // Grab maginalized features
  //    1) select the oldest clone time
  //    2) grab whole the measurements for each feature that contain the given timestamp
  //    3) not delete
  if(state->getEstimationNum(CLONE_CAM0) == params.msckf.max_clone_C) {
    // get oldest clone time
    auto time_marg = state->getMarginalizedTime(CLONE_CAM0);
    // get feature contain the marg time
    tracker->get_feature_database()->features_selected(time_marg, feat_marg, false, true);
  }

  // ------------------------------- Keep clone related measurements -------------------------------- //

  // [0] Get camera clone timestamp  

  auto cam_clones = state->getSubState(CLONE_CAM0);
  std::vector<double> clone_times;
  for(const auto& clone : cam_clones) {
    // convert timestamp string to actual timestamp double
    clone_times.emplace_back(std::stod(clone.first));
  }

  // [1] Clean lost features: 
  //    1) remove non-clone time measurements
  //    2) make sure enough for triangulation 

  auto it0 = feat_lost.begin();
  while (it0 != feat_lost.end()) {
    // clean the feature that don't have the clonetime
    it0->clean_old_measurements(clone_times);
    // count how many measurements
    int num_measurements = 0;
    for (const auto &pair : it0->timestamps) {
      num_measurements += it0->timestamps[pair.first].size();
    }
    //! TODO: if we have DVL-adied triangulation, we don't real need more then 2 features
    // remove feature if not enough for triangulation
    if (num_measurements < 2) {
      it0 = feat_lost.erase(it0);
    } 
    else {
      it0++;
    }
  }

  // [2] Clean marg features: 
  //    1) remove non-clone time measurements
  //    2) make sure enough for triangulation 

  auto it1 = feat_marg.begin();
  while (it1 != feat_marg.end()) {
    // clean the feature that don't have the clonetime
    it1->clean_old_measurements(clone_times);
    // count how many measurements
    int num_measurements = 0;
    for (const auto &pair : it1->timestamps) {
      num_measurements += it1->timestamps[pair.first].size();
    }
    //! TODO: if we have DVL-adied triangulation, we don't real need more then 2 features
    // remove feature if not enough for triangulation
    if (num_measurements < 2) {
      it1 = feat_marg.erase(it1);
    } 
    else {
      it1++;
    }
  }

  if(feat_lost.size() > 0) {
    printf("  Lost feat size=%ld\n", feat_lost.size());
    // for(const auto& f : feat_lost) {
    //   printf("  id:%ld, meas:%ld\n", f.featid, f.timestamps.at(0).size());
    // }
  }

  if(feat_marg.size()>0) {
    printf("  Marg feat size=%ld\n", feat_marg.size());
    // for(const auto& f : feat_marg) {
    //   printf("  id:%ld, meas:%ld\n", f.featid, f.timestamps.at(0).size());
    // }
  }
}


void MsckfManager::selectFeatures(const double time_update, std::vector<Feature> &feat_selected) {
  //! TEST: save all the features in the database
  // recorder->writeString("----- Database -----\n");
  // tracker->get_feature_database()->saveFeatures(recorder);

  // ------------------------------- Grab features ------------------------------------------ //

  // grab lost features
  tracker->get_feature_database()->features_lost(time_update, feat_selected, true, true);

  // grab maginalized features
  double time_marg;
  if(state->getEstimationNum(CLONE_CAM0) == params.msckf.max_clone_C) {

    time_marg = state->getMarginalizedTime(CLONE_CAM0);
    
    tracker->get_feature_database()->features_marginalized(time_marg, time_update, feat_selected, true, true);
  }

  //! TEST: save all selected features
  // std::string str = "----- marg: " + toCloneStamp(time_marg) + " update: " + toCloneStamp(time_update) + " -----\n";
  // recorder->writeString(str);
  // for( auto& i :feat_selected) {
  //   recorder->writeFeature(i);
  // }

  //! TEST: save all the features in the database
  // recorder->writeString("----- Database -----\n");
  // tracker->get_feature_database()->saveFeatures(recorder);

  // ------------------------------- Select features ------------------------------------------ //
  // Sort based on track length: features at the end are long-tracks
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  std::sort(feat_selected.begin(), feat_selected.end(), [](const Feature a, const Feature b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a.timestamps)
      asize += pair.second.size();
    for (const auto &pair : b.timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  }); 

  // select the longest tracked feature if in limited computational resources device
  if ((int)feat_selected.size() > params.msckf.max_msckf_update){
    printf("Manager warning: too many features for update, deleted %ld\n", 
        feat_selected.size() - params.msckf.max_msckf_update);
        
    feat_selected.erase(feat_selected.begin(), feat_selected.end() - params.msckf.max_msckf_update);
  }
}

} // namespace msckf_dvio
