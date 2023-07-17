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
    case InitMode::INIT_SETTING: {
      initializer = std::shared_ptr<InitSetting>(new InitSetting(params.init));
      break;
    }

    case InitMode::INIT_STATIC: {
      break;
    }

    case InitMode::INIT_DVL_PRESSURE: {
      initializer = std::shared_ptr<InitDvlAided>(new InitDvlAided(params.init, params.prior_imu, params.prior_dvl));
      break;
    }

    default:
      break;
  }

  //// setup predictor
  predictor = std::make_shared<Predictor>(params.prior_imu);

  //// setup updater
  updater = std::make_shared<Updater>(params);

  //// setup tracker if system has visual measurements
  if(checkVisual()) {
    // get params
    std::map<size_t, bool> camera_fisheye;
    std::map<size_t, Eigen::VectorXd> camera_calibration;
    std::map<size_t, std::pair<int, int>> camera_wh;

    Eigen::VectorXd cam0_calib;
    auto intrinsic_size = params.prior_cam.intrinsics.size();
    auto distortion_size = params.prior_cam.distortion_coeffs.size();
    cam0_calib.resize(intrinsic_size + distortion_size);
    // convert intrinsics and 
    for(size_t i=0; i<intrinsic_size; i++) {
      cam0_calib(i) = params.prior_cam.intrinsics.at(i) * params.tracking.basic.downsample_ratio;
    }
    // convert distortion_coeffs
    for(size_t i=4; i< (intrinsic_size + distortion_size); i++) {
      cam0_calib(i) = params.prior_cam.distortion_coeffs.at(i-4);
    }

    // setup param for tracker
    if(params.tracking.basic.cam_id == 0){
      camera_fisheye.insert({0, false});
      camera_calibration.insert({0, cam0_calib});
      camera_wh.insert({0, params.prior_cam.image_wh});
    }

    // start tracker
    switch(params.tracking.basic.mode) {
      case TrackMode::TRACK_KLT: {
        tracker = std::shared_ptr<TrackBase>(new TrackKLT (
          params.tracking.klt.num_pts, params.tracking.basic.num_aruco, 
          params.tracking.klt.fast_threshold, params.tracking.klt.grid_x, 
          params.tracking.klt.grid_y, params.tracking.klt.min_px_dist, 
          params.tracking.klt.pyram, params.tracking.basic.img_enhancement));

        tracker->set_calibration(camera_calibration, camera_fisheye);

        break;
      }

      case TrackMode::TRACK_FEATURE: {
        tracker = std::shared_ptr<TrackBase>(new TrackFeature (
          params.tracking.basic.num_aruco, camera_wh));

        tracker->set_calibration(camera_calibration, camera_fisheye);

        break;
      }

      default:
        break;
    }
  }

  frame_count = params.keyframe.frame_count;
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
  //!  correct timeoffset: basic substract time transmit from DVl to bottom, check duraction field of df21 ?

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
  if(!initializer->isInit() && initializer->useSensor(Sensor::PRESSURE)) {
    initializer->feedPressure(data);
  }
}

void MsckfManager::feedDvlCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data) {
  if(!initializer->isInit())
    return;

  std::unique_lock<std::mutex> lck(mtx);

  buf_dvl_pc.emplace_back(data);

  // prevent buffer overflow
  auto frame = std::find_if(buf_dvl_pc.begin(), buf_dvl_pc.end(), [&](const auto& pc) {
                  return (buf_dvl_pc.back()->header.stamp - pc->header.stamp) / 1000000.0 
                          <= params.sys.buffers[Sensor::DVL_CLOUD]; });

  if (frame != buf_dvl_pc.end())                    
    buf_dvl_pc.erase(buf_dvl_pc.begin(), frame);
}

void MsckfManager::feedFeature(FeatureMsg &data) {

  if(!initializer->isInit())
    return;

  tracker->feed_features(data);

  std::unique_lock<std::mutex> lck(mtx);

  buffer_time_img.emplace(data.time);
  if(buffer_time_img.size() > 1000) {
    buffer_time_img.pop();
    printf("warning: feature time msg buffer overflow !\n");
  }

}

void MsckfManager::feedCamera(ImageMsg &data) {
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

  //! TODO: add update timestamp in the tracker
  //! TODO: no need another buffer to store the image time

  // append sensor timestamp to buffer
  buffer_time_img.emplace(data.time);
  if(buffer_time_img.size() > 1000) {
    buffer_time_img.pop();
    printf("warning: camera time msg buffer overflow !\n");
  }

  mtx.unlock();
}

std::vector<pcl::PointCloud<pcl::PointXYZ>> MsckfManager::getDvlCloud(double timestamp, int num) {
  std::unique_lock<std::mutex> lck(mtx);

  // prepare a serial of data with a delta time
  std::vector<std::pair<double, pcl::PointCloud<pcl::PointXYZ>::Ptr>> temp_queue;

  for (auto it = buf_dvl_pc.rbegin(); it != buf_dvl_pc.rend(); ++it)
  {
    auto dt = abs(timestamp - (*it)->header.stamp/1000000.0);
    temp_queue.push_back(std::make_pair(dt,*it));
  }

  // sort increasing order
  std::sort(temp_queue.begin(), temp_queue.end());

  // determine the copy size, in case the buffer size less then given num
  int copy_size = temp_queue.size() < num ? temp_queue.size() : num;

  // copy
  std::vector<pcl::PointCloud<pcl::PointXYZ>> sorted_pc;
  for(size_t i = 0; i < copy_size; i++) {
    sorted_pc.push_back(*temp_queue.at(i).second);
  }

  return sorted_pc;
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

    //! TODO: some sensor not used for initialization, 
    //!       may also clean buffer before the initialization timestamp 
    for (const auto& [sensor, timestamp] : timelines) {
      switch(sensor) {
        case Sensor::IMU: {
          releaseImuBuffer(timestamp);
          break;
        }

        case Sensor::DVL: {
          releaseDvlBuffer(timestamp);
          break;
        }

        case Sensor::PRESSURE: {
          releasePressureBuffer(timestamp);
          break;
        }

        default:
          break;
      }
    }

  }


/**************************************************************************************/
/***************************** Update for multi-sensors *******************************/
/**************************************************************************************/

  //! TODO: check efficiency on "swith" or "if-else"

  switch(selectUpdateSensor()) {

    // choose DVL BT velocity to update IMU
    case DVL: {

      // standard one
      // doDVL();

      // individual BT velocity update
      doBtUpdate();

      break;
    }

    // choose DVL CP pressure to update IMU
    case PRESSURE: {
      // standard one
      // doDVL();

      // only pressure update
      doPressureUpdate();

      // do interplated BT and pressure 
      // doBtPressureUpdate();
      
      break;
    }

    // choose Camera to update IMU
    case CAM0: {

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

  mtx.unlock();

  return update_sensor;
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
    printf("\nManger warning: new image time:%f "
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
  printf("\ncam t: %.9f\n",time_curr_sensor);

  //// [0] IMU Propagation
  predictor->propagate(state, selected_imu);
  // assert(!state->foundSPD("cam_propagate"));
  if(state->foundSPD("cam_propagate")){
    std::exit(EXIT_FAILURE);
  }

  //// [1] State Augmentation: 
  Eigen::Vector3d w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);

  // check if relative motion is larger enough to insert a clone
  // get last clone pose, also meaning key-frame during feature tracking
  bool check_feature = checkFeatures(time_curr_sensor);
  bool check_motion = checkMotion();
  bool check_scene = checkScene(time_curr_sensor);
  bool check_adaptive = checkAdaptive();

  bool insert_clone = check_feature && (check_motion && check_scene);
  // bool insert_clone = check_feature && (check_motion || check_scene);

  // use Last angular velocity for cloning when estimating time offset)
  if(params.msckf.max_clone_C > 0 && insert_clone) {
    // clone IMU pose, augment covariance
    predictor->augment(CAM0, CLONE_CAM0, state, time_curr_sensor, w_I);

    printf("[TEST]: new keyframe:%d\n", state->getEstimationNum(CLONE_CAM0));
  }

  std::vector<Feature> feature_keyframe;
  selectFeaturesKeyFrame(time_curr_sensor, feature_keyframe);

  // [2] DVL-enhanced depth
  enhanceDepth(feature_keyframe);

  // [3] Camera Feature Update: feature triangulation, feature update 
  // feature triangulation
  updater->featureTriangulation(state,feature_keyframe);

  // [4] keep marginalization feature measurements
  selectMargMeasurement(feature_keyframe);

  // update triangulation to the database
  // tracker->get_feature_database()->update_new_triangulation(feature_msckf);

  // do the camera update
  // updater->updateCam(state, feature_msckf);
  updater->updateCamPart(state, feature_keyframe);

  // setup new MSCKF features for visualization
  setFeatures(feature_keyframe);

  //// [4] Marginalization(if reach max clone): 
  if((params.msckf.max_clone_C > 0) &&
     (params.msckf.max_clone_C == state->getEstimationNum(CLONE_CAM0))) {

    // remove used feature measurements:
    //   lost feature measurements: alrady remove when we select
    //   marg feature measurements: delete right now with those used in the update
    tracker->get_feature_database()->cleanup_marg_measurements(feature_keyframe);

    // remove specific marg pose index
    for(size_t i=0; i<params.msckf.marg_pose_index.size(); i++) {
      auto index = params.msckf.marg_pose_index.at(i) - i;
      updater->marginalize(state, CLONE_CAM0, index);
      printf("======= marg index clone\n");
    }
  }

  // [5] cleanup
  if(state->getEstimationNum(CLONE_CAM0) > 0) {
    // remove the oldest pose if no measurements exists

    // get oldest clone timestamp
    auto oldest_time = state->getCloneTime(CLONE_CAM0, 0);

    // find feature measurements at last keyframe timestamp
    std::vector<Feature> oldest_feat;
    tracker->get_feature_database()->features_measurement_selected(oldest_time, oldest_feat, true);
    if(oldest_feat.size() == 0) {
        updater->marginalize(state, CLONE_CAM0, 0);
        printf("======= marg oldest clone\n");
    }

    // oldest_time = state->getCloneTime(CLONE_CAM0, 0);
    // tracker->get_feature_database()->cleanup_out_measurements(oldest_time);
  }

  if(state->getEstimationNum(CLONE_CAM0) > 0) {
    // remove outdate feature measurements: remove older then oldest clone timestamp 
    auto oldest_time = state->getCloneTime(CLONE_CAM0, 0);
    tracker->get_feature_database()->cleanup_out_measurements(oldest_time);
  }

}

void MsckfManager::doBtUpdate() {

  // [0] get data
  DvlMsg selected_dvl;
  std::vector<ImuMsg> selected_imu;

  if(!getImuForBt(selected_dvl, selected_imu)) {
    return;
  }
  printf("\nDVL BT t: %.9f\n",selected_dvl.time);

  // [1] IMU propagation
  predictor->propagate(state, selected_imu);
  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  if(state->foundSPD("dvl_propagate")){
    std::exit(EXIT_FAILURE);
  }

  // [2] update

  // Last angular velocity and linear velocity
  Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
  Eigen::Vector3d last_v_D = selected_dvl.v;

  // update
  updater->updateDvl(state, last_w_I, last_v_D);
}

void MsckfManager::doPressureUpdate() {

  // [0] get data
  PressureMsg selected_pressure;
  std::vector<ImuMsg> selected_imu;
  if(!getImuForPressure(selected_pressure, selected_imu)) {
    return;
  }
  
  printf("\nDVL Pressure t: %.9f\n", selected_pressure.time);

  // [1] IMU propagation
  predictor->propagate(state, selected_imu);
  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  if(state->foundSPD("pressure_propagate")){
    std::exit(EXIT_FAILURE);
  }

  // [2] State Update with Pressure

  // get the begin pressure value
  double pres_init = state->getPressureInit();
  double pres_curr = selected_pressure.p;

  // updater->updatePressureManual(state, pres_init, pres_curr, 1);  
  updater->updatePressureManual(state, pres_init, pres_curr, 2);  
  // updater->updatePressure(state, pres_init, pres_curr);  
}

void MsckfManager::doBtPressureUpdate() {
  // [0] get data
  DvlMsg selected_dvl;
  PressureMsg selected_pressure;
  std::vector<ImuMsg> selected_imu;
  if(!getImuForBtPressure(selected_dvl, selected_pressure, selected_imu)) {
    return;
  }

  printf("\nDVL Pressure t: %.9f\n", selected_pressure.time);

  // [1] IMU propagation
  predictor->propagate(state, selected_imu);
  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  if(state->foundSPD("dvl_pressure_propagate")){
    std::exit(EXIT_FAILURE);
  }

  // [2]  State Update with DVL BT

  // Last angular velocity and linear velocity
  Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
  Eigen::Vector3d last_v_D = selected_dvl.v;

  // update DVL bt 
  updater->updateDvl(state, last_w_I, last_v_D);

  // [3] State Update with DVL Pressure

  // get the begin pressure value
  double pres_init = state->getPressureInit();
  double pres_curr = selected_pressure.p;
  // update
  updater->updatePressure(state, pres_init, pres_curr);  
  // updater->updatePressureManual(state, pres_init, pres_curr, 2);  
}

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
     buffer_pressure.front().time <= state->getTimestamp()){
    //// this pressure timetsamp is wrong: maybe delay by ros callback, should be process by previous velocity

    printf("Manager warning: pressure time:%f is before state time:%f\n, drop it now!", 
            buffer_pressure.front().time, state->getTimestamp());
    buffer_pressure.erase(buffer_pressure.begin());
  }
  else if(buffer_pressure.size() > 0 && buffer_dvl.size() > 0 &&
          buffer_pressure.front().time < buffer_dvl.front().time){
    printf("\ndo intepolated velocity-pressure \n");

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
    printf("\ndo pressure + velocty\n");

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
          buffer_dvl.size() > 0)  {
    printf("\ndo pure velocty\n");

    //// pure DVL update

    // get data
    new_dvl = buffer_dvl.front();
    do_velocity = true;


    // clean buffer
    last_dvl = buffer_dvl.front(); 
    erase_velocity = true;
  }

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

  mtx.unlock();

  /******************** imu propagation + velocity update + pressure update(if)********************/

  if(selected_imu.size()>0){
    // IMU propagation
    predictor->propagate(state, selected_imu);
    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    // assert(!state->foundSPD("dvl_propagate"));
    if(state->foundSPD("dvl_propagate")){
      std::exit(EXIT_FAILURE);
    }

    // just for pure IMU 
    if(do_velocity){

      // Last angular velocity and linear velocity
      Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
      Eigen::Vector3d last_v_D = new_dvl.v;

      // update
      updater->updateDvl(state, last_w_I, last_v_D);
    }

    if(do_pressure){
      // get the begin pressure value
      double pres_init = state->getPressureInit();
      double pres_curr = new_pres.p;

      // update
      updater->updatePressure(state, pres_init, pres_curr);
      // updater->updatePressureManual(state, pres_init, pres_curr, 2);  
    }
    
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
    const double time_update, std::vector<Feature> &feat_keyframe) {
  // ------------------------------- Grab features ------------------------------------------ //

  // Grab lost features:
  //    1) grab whole the measuremens for each lost feature
  //    2) delete the grabbed features in the database
  tracker->get_feature_database()->features_lost(time_update, feat_keyframe, true, true);

  // Grab maginalized features
  //    1) select the oldest clone time
  //    2) grab whole the measurements for each feature that contain the given timestamp
  //    3) not delete
  if(state->getEstimationNum(CLONE_CAM0) == params.msckf.max_clone_C) {
    // Grab index 0 of slide window to get oldest clone time
    auto time_oldest = state->getCloneTime(CLONE_CAM0, 0);
    // get feature contain this marg time
    tracker->get_feature_database()->features_selected(time_oldest, feat_keyframe, false, true);
  }

  // --------------------------- Filter non-keyframe or bad measurements -------------------------- //

  // get clone timestamp
  std::vector<double> clone_times;
  auto clone_num = state->getEstimationNum(CLONE_CAM0);

  for( size_t index = 0; index < clone_num; index++ ){
    clone_times.push_back(state->getCloneTime(CLONE_CAM0, index));
  }

  // filter features: 
  auto it0 = feat_keyframe.begin();
  while (it0 != feat_keyframe.end()) {

    // [0] Clean non-clone stamp measurements
    it0->clean_old_measurements(clone_times);

    // [1] Check: at least 2 measurements can be used for update
    int num_measurements = 0;
    for (const auto &pair : it0->timestamps) {
      num_measurements += it0->timestamps[pair.first].size();
    }                  

    if(num_measurements < 2) {
      it0 = feat_keyframe.erase(it0);
      continue;
    }

    it0++;
  }

  // --------------------------- Determine the anchor -------------------------- //

  // loop each feature
  for(auto& feat : feat_keyframe) {

    double distance = std::numeric_limits<double>::max();

    // loop each measurement
    for(size_t i=0; i<feat.uvs_norm.at(0).size(); i++) {
      auto d = sqrt(feat.uvs_norm.at(0).at(i)(0) * feat.uvs_norm.at(0).at(i)(0) +
                    feat.uvs_norm.at(0).at(i)(1) * feat.uvs_norm.at(0).at(i)(1));

      // get the mini distance 
      if(d <= distance) {
        distance = d;
        // reset the anchor of feature for this update
        feat.anchor_clone_timestamp = feat.timestamps.at(0).at(i);
        feat.anchor_clone_depth = 0;
      }
    }
  }

}

void MsckfManager::enhanceDepth(std::vector<Feature> &features) {

  // get extrinsic calibration for IMU and DVL
  Eigen::Matrix3d R_I_D = params.msckf.do_R_I_D ? 
                          toRotationMatrix(state->getEstimationValue(DVL,EST_QUATERNION)) :
                          toRotationMatrix(params.prior_dvl.extrinsics.head(4)); 
  Eigen::Vector3d p_I_D = params.msckf.do_p_I_D ? 
                          state->getEstimationValue(DVL,EST_POSITION) : 
                          params.prior_dvl.extrinsics.tail(3);
  Eigen::Matrix4d T_I_D = Eigen::Matrix4d::Identity();
  T_I_D.block(0,0,3,3) = R_I_D;
  T_I_D.block(0,3,3,1) = p_I_D;

  // get extrinsic calibration for IMU and Camera
  Eigen::Matrix3d R_C_I = params.msckf.do_R_C_I ?
                          toRotationMatrix(state->getEstimationValue(CAM0,EST_QUATERNION)) :
                          toRotationMatrix(params.prior_cam.extrinsics.head(4));
  Eigen::Vector3d p_C_I = params.msckf.do_p_C_I ?
                          state->getEstimationValue(CAM0,EST_POSITION) :
                          params.prior_cam.extrinsics.tail(3);
  Eigen::Matrix4d T_C_I = Eigen::Matrix4d::Identity();
  T_C_I.block(0,0,3,3) = R_C_I;
  T_C_I.block(0,3,3,1) = p_C_I;

  // loop each feature
  for(auto& feat : features) {
    // get matched pointclouds based on anchor timestamp
    auto matched_pcs = getDvlCloud(feat.anchor_clone_timestamp, params.enhancement.matched_num);

    // check with pointcloud until get depth estimation
    for(const auto& pc : matched_pcs) {
      // [1] Interpolate IMU pose at DVL timetsmap 

      //! TODO: add time sync, now asssuming DVL and IMU are sync
      auto t_I = pc.header.stamp / 1000000.0;

      Eigen::Matrix3d R_I_G;
      Eigen::Vector3d p_G_I;
      if(!poseInterpolation(t_I, R_I_G, p_G_I)) {
        continue;
      }

      // [2] Filter bad DVL pointcloud in case the multi-path

      // get transform between DVL and global
      Eigen::Matrix4d T_G_I = Eigen::Matrix4d::Identity();
      T_G_I.block(0,0,3,3) = R_I_G.transpose();
      T_G_I.block(0,3,3,1) = p_G_I;
      Eigen::Matrix4d T_G_D =  T_G_I * T_I_D;

      // transform to global frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_G(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud (pc, *pc_G, T_G_D);

      // filter bad pointcloud: less then 4 points
      if(pc_G->points.size() != 4) {
        continue;
      }

      // filter bad pointcloud: based on Standard Deviation of global Z
      double mean = 0;
      for(auto it = pc_G->begin(); it != pc_G->end(); it++){
        mean += it->z;
      }
      mean /= pc_G->points.size();

      double sd = 0;
      for(auto it = pc_G->begin(); it != pc_G->end(); it++){
        sd += (it->z - mean) * (it->z - mean);
      }      
      sd = sqrt(sd / (pc_G->points.size() - 1));

      if(sd > params.enhancement.standard_deviation) {
        // printf("Bad DVL points, sd=%f, z=[%f,%f,%f,%f]\n", sd,
          // pc_G->points[0].z,pc_G->points[1].z,pc_G->points[2].z,pc_G->points[3].z);
        continue;
      }

      // [3] check if inside 4 DVL point coverage

      // get anchor pose from clone state 
      auto anchor_time = toCloneStamp(feat.anchor_clone_timestamp);
      auto pose_vec = state->getEstimationValue(CLONE_CAM0, anchor_time);
      Eigen::Matrix3d R_Ia_G = toRotationMatrix(pose_vec.head(4));
      Eigen::Vector3d p_G_Ia = pose_vec.tail(3);

      // transform to camera frame
      Eigen::Matrix4d T_Ia_G = Eigen::Matrix4d::Identity();
      T_Ia_G.block(0,0,3,3) = R_Ia_G;
      T_Ia_G.block(0,3,3,1) = - R_Ia_G * p_G_Ia;

      // p_C = T_I_C^-1 * T_G_I^-1 * p_G      
      Eigen::Matrix4d T_C_G = T_C_I * T_Ia_G;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_C(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud (*pc_G, *pc_C, T_C_G);

      // construct a polygon on normalized plane of camera using DVL 4 points
      std::vector<Eigen::Vector2d> polygon;
      std::vector<Eigen::Vector3d> quadrilateral;

      for(auto it = pc_C->begin(); it != pc_C->end(); it++) {
        auto x_norm = it->x/it->z;
        auto y_norm = it->y/it->z;
        polygon.push_back(Eigen::Vector2d(x_norm, y_norm));
        quadrilateral.push_back(Eigen::Vector3d(x_norm, y_norm, it->z));
      }

      // get un_norm for anchor frame
      Eigen::Vector2d uv_norm = Eigen::Vector2d::Zero();
      auto it = std::find_if(feat.timestamps.at(0).begin(), feat.timestamps.at(0).end(),
                  [&](const auto& t){return t == feat.anchor_clone_timestamp ;});
      if(it != feat.timestamps.at(0).end()) {
          uv_norm(0) = feat.uvs_norm.at(0).at(it - feat.timestamps.at(0).begin())(0);
          uv_norm(1) = feat.uvs_norm.at(0).at(it - feat.timestamps.at(0).begin())(1);
      }

      // check
      if(!pointInPolygon(polygon, uv_norm)) {
        // printf("Depth enahcement: not covered\n");
        // std::cout<<"vertex: "<< polygon[0].transpose()<<","
        //                      << polygon[1].transpose()<<","
        //                      << polygon[2].transpose()<<","
        //                      << polygon[3].transpose()<<"\n";
        // std::cout<<"uv_norm: "<<uv_norm.transpose()<<std::endl;
        continue;
      }

      // [4] bilinear interpolation
      double interpolated_depth;
      if(!bilinearInterpolation(quadrilateral, uv_norm, interpolated_depth)) {
        continue;
      }

      // found, then break
      feat.anchor_clone_depth = interpolated_depth;

      // addMatchedPointcloud(pc_C, feat.anchor_clone_timestamp, "left_camera");
      addMatchedPointcloud(pc_G, t_I, "odom");

      break;
    }
  }
}

bool MsckfManager::poseInterpolation(double timestamp, Eigen::Matrix3d &R, Eigen::Vector3d &p) {
  // check if given timestamp exist in the middle of [t_a, t_b] interval

  auto clone_num = state->getEstimationNum(CLONE_CAM0);

  // loop interval for two consecutive clones
  for( size_t index = 0; index < clone_num - 1; index++ ){
    // get a interval
    auto t_a = state->getCloneTime(CLONE_CAM0, index);
    auto t_b = state->getCloneTime(CLONE_CAM0, index + 1);

    // check if exist
    if(timestamp >= t_a && timestamp <= t_b) {
      // get R,p
      auto pose_a = state->getClonePose(CLONE_CAM0, index);
      auto pose_b = state->getClonePose(CLONE_CAM0, index + 1);

      std::tie(R,p) = interpolatePose(pose_a, t_a, pose_b, t_b, timestamp);

      return true;
    }
  }

  return false;
}

void MsckfManager::selectMargMeasurement(std::vector<Feature> &feat_keyframe) {
  // if we are not reach the max slide window size, we wait
  if(state->getEstimationNum(CLONE_CAM0) < params.msckf.max_clone_C) 
    return;

  // get actually need marginalize out measurements index timestamp
  std::vector<double> clone_times;
  for(const auto& index : params.msckf.marg_meas_index) {
      clone_times.push_back(
        state->getCloneTime(CLONE_CAM0,index));
  }

  // get max-tracked features
  std::vector<Feature> feat_marg;
  auto time_marg = state->getCloneTime(CLONE_CAM0, 0);
  tracker->get_feature_database()->features_selected(time_marg, feat_marg, false, true);

  // only keep marg index measurements
  for(const auto& f : feat_marg) {
    // check if the marg feature exist in our selected feature pool
    auto frame = std::find_if(feat_keyframe.begin(), feat_keyframe.end(),
                  [&](const auto& feat){return feat.featid == f.featid ;});
    // find                 
    if(frame != feat_keyframe.end()) {
      // only keep the marg index measurements
      frame->clean_old_measurements(clone_times);

      // check if measurements less then 2
      int num_measurements = 0;
      for (const auto &pair : frame->timestamps) {
        num_measurements += frame->timestamps[pair.first].size();
      }

      if(num_measurements < 2) {
        feat_keyframe.erase(frame);
      }      
    }        
  }

}

bool MsckfManager::getImuForBt(DvlMsg &dvl_msg,  std::vector<ImuMsg> &imu_msgs) {
  std::unique_lock<std::mutex> lck(mtx);

  // [0] select DVL
  dvl_msg = buffer_dvl.front();

  // [1] select IMU time duration

  // convert sensor time into IMU time
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
                params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = dvl_msg.time + offset;

  // make sure sensor data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("\nManger warning: new dvl measurement time:%f" 
            "is eariler then last state time:%f, drop it now!\n",
            time_curr_state, time_prev_state);

    buffer_dvl.erase(buffer_dvl.begin());

    return false;
  }

  // [2] select IMU data: make sure IMU time > current sensor time for interpolation
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                   [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    // select
    imu_msgs = selectImu(time_prev_state, time_curr_state);

    // erase selected dvl 
    last_dvl = buffer_dvl.front(); 
    buffer_dvl.erase(buffer_dvl.begin());

    // erase selected IMU, but leave one for next interpolate
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);

    return true;   
  }
  else {
    return false;
  }
}

bool MsckfManager::getImuForPressure(PressureMsg &pressure_msg, std::vector<ImuMsg> &imu_msgs) {
  std::unique_lock<std::mutex> lck(mtx);

  // [0] select pressure 
  pressure_msg = buffer_pressure.front();

  // [1] select IMU time duration

  // convert sensor time into IMU time
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
                params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = pressure_msg.time + offset;

  // make sure sensor data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("\nManger warning: new pressure measurement time:%f" 
            "is eariler then last state time:%f, drop it now!\n",
            time_curr_state, time_prev_state);

    buffer_pressure.erase(buffer_pressure.begin());

    return false;
  }

  // [2] select IMU data: make sure IMU time > current sensor time for interpolation
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                   [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    // select
    imu_msgs = selectImu(time_prev_state, time_curr_state);

    // erase selected pressure 
    buffer_pressure.erase(buffer_pressure.begin());

    // erase selected IMU, but leave one for next interpolate
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);

    return true;   
  }
  else {
    return false;
  }

}

bool MsckfManager::getImuForBtPressure(DvlMsg &dvl_msg,  PressureMsg &pressure_msg, std::vector<ImuMsg> &imu_msgs) {
  std::unique_lock<std::mutex> lck(mtx);

  // [0] check if DVL for interpolation avaiable
  if(buffer_dvl.size() < 0 || 
     buffer_pressure.front().time > buffer_dvl.front().time){
    return false;
  }

  // [1] select pressure 
  pressure_msg = buffer_pressure.front();

  // [2] select DVL: pressure inside last and next DVL
  if(pressure_msg.time > last_dvl.time && 
     (pressure_msg.time < buffer_dvl.front().time || 
      pressure_msg.time == buffer_dvl.front().time) ) {
    dvl_msg = interpolateDvl(last_dvl, buffer_dvl.front(), pressure_msg.time);
  }
  else {
    // send warning
    printf("Manager warning: pressure interpolated failed, "
           "last t:%f, interpoated t:%f, next t:%f, will drop this pressure\n",
           last_dvl.time, pressure_msg.time, buffer_dvl.front().time);
    // delete bad pressure
    buffer_pressure.erase(buffer_pressure.begin());

    return false;
  }

  // [3] select IMU time duration
  // convert sensor time into IMU time
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : 
                params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = pressure_msg.time + offset;
  // make sure sensor data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new pressure measurement time:%f" 
            "is eariler then current state time:%f\n",
            time_curr_state, time_prev_state);

    buffer_pressure.erase(buffer_pressure.begin());
    return false;
  }

  // [4] select IMU data: make sure IMU time > current sensor time for interpolation
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                   [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    // select imu
    imu_msgs = selectImu(time_prev_state, time_curr_state);

    //erase selected DVL if velocity and pressure at same timestamp(e.g. both from DVL BT)
    if(buffer_pressure.front().time == buffer_dvl.front().time) {
      last_dvl = buffer_dvl.front(); 
      buffer_dvl.erase(buffer_dvl.begin());  
    }

    // erase selected pressure
    buffer_pressure.erase(buffer_pressure.begin());

    // erase selected IMU, but leave one for next interpolate
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);     

    return true;  
  }
  else {
    return false;
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

void MsckfManager::setFeatures(std::vector<Feature> &features) {
  std::unique_lock<std::mutex> lck(mtx);

  // get feature
  for (size_t f = 0; f < features.size(); f++) {
    // setup feature position: enhaced + not enhanced
    Eigen::Vector3d color;
    if(features[f].anchor_clone_depth == 0) {
      // red for normal triangulation
      color << 255,0,0;
    }
    else {
      // green for depth enhanced
      color << 0,255,0;
    }
    trig_feat.emplace_back(std::make_tuple(features[f].p_FinG, color));

    // setup the original features
    // trig_feat.emplace_back(std::make_tuple(features[f].p_FinG_original, 
    //                                        Eigen::Vector3d(255,0,0)));
  }
}

std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> MsckfManager::getFeatures() {
  std::unique_lock<std::mutex> lck(mtx);

  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> out_trig_feat;

  // copy triangulated message
  std::copy(trig_feat.begin(), trig_feat.end(), std::back_inserter(out_trig_feat)); 

  // clean
  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>>().swap(trig_feat);

  return out_trig_feat;
}

bool MsckfManager::checkScene(double curr_time) {

  // [0] Get last keyframe feature (associated with last clone pose)

  // if no clone exist, then insert current one
  auto clone_size = state->getEstimationNum(CLONE_CAM0);
  if(clone_size == 0) {
    return true;
  }

  // get latest clone timestamp
  auto clone_time = state->getCloneTime(CLONE_CAM0, clone_size-1);

  // find feature measurements at last keyframe timestamp
  std::vector<Feature> key_frame_feat;
  tracker->get_feature_database()->features_measurement_selected(clone_time, key_frame_feat, true);

  // [1] Get current frame feature

  // find feature measurements at current frame timestamp
  std::vector<Feature> curr_frame_feat;
  tracker->get_feature_database()->features_measurement_selected(curr_time, curr_frame_feat, true);

  // [2] Compare

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

  // if(ratio < params.keyframe.scene_ratio) {
  if(ratio <= params.keyframe.scene_ratio) {
    return true;
  }
  else {
    // printf("ratio:%f\n", ratio);
    return false;
  }
}

bool MsckfManager::checkFeatures(double curr_time) {
  // find feature measurements at current frame timestamp
  std::vector<Feature> curr_frame_feat;
  tracker->get_feature_database()->features_measurement_selected(curr_time, curr_frame_feat, true);

  // only the detected feature reach the minimum requirement 
  if(curr_frame_feat.size() < params.keyframe.min_tracked) {
    return false;
  }
  else {
    return true;
  }
}

bool MsckfManager::checkAdaptive() {
  // [0] if no clone exist, then insert current one
  auto clone_size = state->getEstimationNum(CLONE_CAM0);
  if(clone_size == 0) {
    return true;
  }

  // [1] Get constrain 

  // get current vehilce velocity
  Eigen::Vector3d v_G_I = state->getEstimationValue(IMU, EST_VELOCITY);

  double v_xy = sqrt(v_G_I(0)*v_G_I(0) + v_G_I(1)*v_G_I(1));

  // get constrain
  double D = params.keyframe.adaptive_factor * pow(v_xy, params.keyframe.adaptive_power);

  // [2] Get distance between current frame and latest keyframe

  // get latest clone pose 
  Eigen::VectorXd pose = state->getClonePose(CLONE_CAM0, clone_size-1);   
  Eigen::Matrix3d R_Ii_G = toRotationMatrix(pose.block(0,0,4,1));
  Eigen::Vector3d p_G_Ii = pose.block(4,0,3,1);

  // get current IMU pose
  Eigen::Vector3d p_G_Ij = state->getEstimationValue(IMU, EST_POSITION);

  // get 2D or 3D distance
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

  if(distance >= D) {
    // printf("adaptive: yes\n");
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
} // namespace msckf_dvio
