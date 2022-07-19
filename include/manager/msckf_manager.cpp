#include "manager/msckf_manager.h"


namespace msckf_dvio
{

MsckfManager::MsckfManager(Params &parameters) : is_odom(false)
{
  // init variable
  this->params = parameters;

  state = std::make_shared<State>(params);

  //// setup imu initializer
  initializer = std::shared_ptr<InitDvlAided>(new InitDvlAided(params.init, params.prior_imu, params.prior_dvl));
                                                      
  //// setup predictor
  predictor = std::make_shared<Predictor>(params.prior_imu);

  //// setup updater
  updater = std::make_shared<Updater>(params.prior_dvl, params.prior_cam, params.msckf);

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

}

void MsckfManager::feedImu(const ImuMsg &data) {
  //// append to the buffer 
  buffer_mutex.lock();
  buffer_imu.emplace_back(data);
  buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  if(!initializer->isInit())
    initializer->feedImu(data);

  //!TODO: delete imu messages that are older then some time, 
  //!      like 10s in case 
}

void MsckfManager::feedDvl(const DvlMsg &data) {
  //! TODO: preprocess dvl:
  //!  1) correct timeoffset: basic substract time transmit from DVl to bottom, check duraction field of df21
  //!  2) remove spikes noise (Median filtering) and data smoothing (Moving Average Filter)

  //// append to the buffer 
  buffer_mutex.lock();
  buffer_dvl.emplace_back(data);

  // if(buffer_dvl.size()>200){
  //   printf("Manager warning: DVL velocity buffer overflow, drop now!\n");
  //   buffer_dvl.erase(buffer_dvl.begin());
  // }

  buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  if(!initializer->isInit())
    initializer->feedDvl(data);
}

void MsckfManager::feedPressure(const PressureMsg &data) {
  //// append to the buffer 
  buffer_mutex.unlock();
  buffer_pressure.emplace_back(data);

  // if(buffer_pressure.size()>600){
  //   printf("Manager warning: pressure buffer overflow, drop now!\n");
  //   buffer_pressure.erase(buffer_pressure.begin());
  // }
  buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  //! TODO: check if this sensor is need for initialization
  //! initializer->needSensor(PRESSURE) ?
  if(!initializer->isInit())
    initializer->feedPressure(data);
}

void MsckfManager::feedCamera(ImageMsg &data) {
  // check if system is initialized 

  //! TODO: need to store image if this not used initialization
  if(!initializer->isInit())
    return;

  // system initialized append to sensor buffer
  buffer_mutex.unlock();

  buffer_time_img.emplace(data.time);

  buffer_mutex.unlock();

  // do front-end tracking for this sensor
  tracker->feed_monocular(data.time, data.image, 0);

  //// [time, features(lost+marg)]

  //// clean database

  // std::shared_ptr<FeatureDatabase> database = tracker->get_feature_database();
  // std::vector<std::shared_ptr<Feature>> feats_lost = database->features_not_containing_newer(data.time);

  // // delete access features
  // for (auto const &f : feats_lost) {
  //   f->to_delete = true;
  // }
  // database->cleanup();

  // visualization
  cv::Mat img_history;
  tracker->display_history(img_history, 0, 255, 255, 255, 255, 255);

  // store tracked images
  ImageMsg msg;
  msg.image = img_history;
  msg.time = data.time;
  
  buffer_mutex.unlock();
  tracked_img.emplace(msg);
  buffer_mutex.unlock();
}

void MsckfManager::backend() {

/**************************************************************************************/
/****************************** Sysmtem initailization ********************************/
/**************************************************************************************/

  if(!initializer->isInit()) {

    initializer->checkInit();

    if(initializer->isInit()){
      std::vector<double> data_time;
      initializer->updateInit(state, params, data_time);

      printf("\n+++++++++++++++\n");

      //// clean manager data buffer which used in initialization
      buffer_mutex.lock();
      
      // delete IMU used for initialization
      auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                    [&](const auto& imu){return imu.time == data_time.at(0) ;});
      if (frame_imu != buffer_imu.end())
        buffer_imu.erase(buffer_imu.begin(), frame_imu);

      // delete DVL BT Velocity used for initialization
      printf("dvl init time:%f, buffer end time:%f, buffer size=%ld\n",
            data_time.at(1), buffer_dvl.at(buffer_dvl.size()-1).time, buffer_dvl.size());

      auto frame_dvl = std::find_if(buffer_dvl.begin(), buffer_dvl.end(),
                    [&](const auto& dvl){return dvl.time >= data_time.at(1) ;});
      if (frame_dvl != buffer_dvl.end()){
        last_dvl = *(frame_dvl);
        buffer_dvl.erase(buffer_dvl.begin(), frame_dvl);
        printf("after erased:%f\n", buffer_dvl.begin()->time);
      }                    

      // delete DVL pressure used for initialization
      auto frame_pres = std::find_if(buffer_pressure.begin(), buffer_pressure.end(),
                    [&](const auto& pressure){return pressure.time >= data_time.at(2) ;});
      if (frame_pres != buffer_pressure.end())                    
        buffer_pressure.erase(buffer_pressure.begin(), frame_pres);

      buffer_mutex.unlock();
    }
    else
      return;
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

  switch(selectUpdateSensor()) {

    // choose DVL BT velocity to update IMU
    case VELOCITY: 
      doDVL();
      break;

    // // choose DVL CP pressure to update IMU
    // case PRESSURE: 
    //   doPressure();
    //   break;

    // choose Camera to update IMU
    case IMAGE: 
      doCamera();
      break;

    // no sensor data avaiable
    case NONE: 
      break;
  }
}

SensorName MsckfManager::selectUpdateSensor() {
  double early_time = std::numeric_limits<double>::infinity();
  SensorName update_sensor = NONE;

  buffer_mutex.lock();

  //// check Camera buffer
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
      update_sensor = IMAGE;
    }
  }

  //// check DVL velocity buffer
  if(buffer_dvl.size() > 0){
    // get first sensor timestamp into IMU frame
    double time = buffer_dvl.front().time;
    if(params.msckf.do_time_I_D)
      time += state->getEstimationValue(DVL, EST_TIMEOFFSET)(0);
    else
      time += params.prior_dvl.timeoffset;

    // compare with other sensor
    if(time < early_time){
      early_time = time;
      update_sensor = VELOCITY;
    }
  }

  // //// check DVL pressure buffer
  // if(buffer_pressure.size() > 0){
  //   // get first sensor timestamp into IMU frame
  //   double time = buffer_pressure.front().time;
  //   if(params.msckf.do_time_I_D)
  //     time += state->getEstimationValue(DVL, EST_TIMEOFFSET)(0);
  //   else
  //     time += params.prior_dvl.timeoffset;

  //   // compare with other sensor
  //   if(time < early_time){
  //     early_time = time;
  //     update_sensor = PRESSURE;
  //   }
  // }

  buffer_mutex.unlock();

  return update_sensor;
}

void MsckfManager::doCamera() {

  /****************************** grab IMU data ******************************/
  buffer_mutex.lock();

  std::vector<ImuMsg> selected_imu;

  //// determine selected IMU range, from last updated to current sensor measurement
  auto time_offset = params.msckf.do_time_C_I ? 
                     state->getEstimationValue(CAM0, EST_TIMEOFFSET)(0) : 
                     params.prior_cam.timeoffset;
  auto time_curr_sensor = buffer_time_img.front();
  auto time_prev_state = state->getTimestamp();
  auto time_curr_state = time_curr_sensor + time_offset;

  //// make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new image time:%f is eariler then current state time:%f, drop it now!\n",
            time_curr_state, time_prev_state);
  }

  //// make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    //// make sure we have at least 2 IMU data
    selected_imu = selectImu(time_prev_state, time_curr_state);

    // earse selected IMU data
    // leave one IMU data time <= current state time, so we can interploate later
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);  

    // erase selected image data
    buffer_time_img.pop();
  }

  buffer_mutex.unlock();

  /******************** imu propagation + image update ********************/
  if(selected_imu.size()>0){

    // predictor->propagate(state, selected_imu);
    // // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    // assert(!state->foundSPD());

    Eigen::Vector3d w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
    //// use Last angular velocity for cloning when estimating time offset)
    if(params.msckf.max_clone_C > 0)
      predictor->augment(CAM0, CLONE_CAM0, state, time_curr_sensor, w_I);

    std::vector<std::shared_ptr<Feature>> feature_MSCKF = selectFeatures(time_curr_sensor);


    //// feature triangulation, feature update 
    updater->updateCam(state, feature_MSCKF);


    //// clean up system: 

    //// if max clone is reached, do the marginalization: remove the clone and related covarinace
    if( (params.msckf.max_clone_C > 0) &&
        (params.msckf.max_clone_C < state->getEstimationNum(CLONE_CAM0)) ) {

      updater->marginalize(state, CLONE_CAM0);
    }

    //// erase feature pixels that already used for update  

    //// double check open_vins clean what else

    // is_odom = true;
  }

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

  buffer_mutex.lock();

  if(buffer_pressure.size() > 0 && 
     buffer_pressure.front().time < buffer_dvl.front().time &&
     buffer_pressure.front().time <= state->getTimestamp()){
    //// this pressure timetsamp is wrong: maybe dely by ros callback, should be process by previous velocity

    printf("Manager warning: pressure time:%f is before state time:%f\n, drop it now!", 
            buffer_pressure.front().time, state->getTimestamp());
    buffer_pressure.erase(buffer_pressure.begin());
  }
  else if(buffer_pressure.size() > 0 && 
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
  else if(buffer_pressure.size() > 0 &&
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
  else if(buffer_pressure.size() == 0 ||
          buffer_pressure.front().time > buffer_dvl.front().time){
    // printf("do pure velocty\n");

    //// pure DVL update

    // get data
    new_dvl = buffer_dvl.front();
    do_velocity = true;


    // clean buffer
    last_dvl = buffer_dvl.front(); 
    erase_velocity = true;
  }
  else{
    printf("\n---------------\nDVL-BT-Velocity:\n");
    if(buffer_dvl.size()>0){
      for(const auto &data: buffer_dvl)
        printf("t: %f, ", data.time);
    }

    printf("\nDVL-CP-Pressure:\n");
    if(buffer_pressure.size()>0){
      for(const auto &data: buffer_pressure)
        printf("t: %f, ", data.time);
    }
  }


  //! TEST: for only IMU-Velocity MSCKF
  // new_dvl = buffer_dvl.front();
  // do_velocity = true;

  // printf("do IMU propagation + velocty update\n");

  // // clean buffer
  // last_dvl = buffer_dvl.front(); 
  // erase_velocity = true;

  buffer_mutex.unlock();


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
  }

  buffer_mutex.lock();

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
    if(erase_pressure)
      buffer_pressure.erase(buffer_pressure.begin());
    // erase selected DVL BT velocity data
    if(erase_velocity)
      buffer_dvl.erase(buffer_dvl.begin());
    
  }
  // else{
  //   printf("Manger warning: current IMU time:%f not > current sensor time:%f\n", buffer_imu.back().time, time_curr_state);
  // }

  buffer_mutex.unlock();

  /******************** imu propagation + velocity update + pressure update(if)********************/

  if(selected_imu.size()>0){
    if(do_velocity){

      predictor->propagate(state, selected_imu);

      // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
      assert(!state->foundSPD());

      // Last angular velocity and linear velocity
      Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
      Eigen::Vector3d last_v_D = new_dvl.v;

      //// use Last angular velocity for cloning when estimating time offset)
      if(params.msckf.max_clone_D > 0)
        predictor->augmentDvl(state, time_curr_sensor, last_w_I);

      // update
      updater->updateDvl(state, last_w_I, last_v_D);
      // updater->updateDvl(state, last_w_I, last_v_D, true);

      // marginalize 
      // if max clone of DVL is reached, then do the marginalize: remove the clone and related covarinace
      if( (params.msckf.max_clone_D > 0) &&
          (params.msckf.max_clone_D < state->getEstimationNum(CLONE_DVL)) ) {

        updater->marginalize(state, CLONE_DVL);
      }

      is_odom = true;
    }

    if(do_pressure){
      // get the begin pressure value
      double pres_init = state->getPressureInit();
      double pres_curr = new_pres.p;

      // update
      updater->updatePressure(state, pres_init, pres_curr, true);

      // is_odom = true;
    }
  }

}

void MsckfManager::doPressure() {

  /********************* no imu propagation ********************/
  // // select pressure data
  // PressureMsg selected_pres = buffer_pressure.front();
  // buffer_pressure.erase(buffer_pressure.begin());

  // // get the begin pressure value
  // double pres_init = state->getPressureInit();
  // double pres_curr = selected_pres.p;

  // // update
  // updater->updatePressure(state, pres_init, pres_curr, true);

  // is_odom = true;

  /******************** IMU propagation, pressure update ********************/

  //! TODO: deal with pressure from BT, no need propagation
  //// if state time = pressure time: (pressure from BT)
  ////    means pressure data from DVL BT, IMU alredy propagated in DVL update 
  //// if state time < pressure time: (pressure from CP)
  ////    means pressure data from individual pressure sensing(individual pressure sensor or pressure from CP) 

  PressureMsg selected_pres;
  std::vector<ImuMsg> selected_imu;

  buffer_mutex.lock();

  //// select first pressure data
  selected_pres = buffer_pressure.front();

  //// deal with pressure from CP
  //// determine selected IMU range, from last updated to current sensor measurement
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = selected_pres.time + offset;

  //// make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new pressure measurement time:%f is eariler then current state time:%f\n",
            time_curr_state, time_prev_state);
    std::exit(EXIT_FAILURE);
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

  buffer_mutex.unlock();


  /******************** do pressure msckf update ********************/

  if(selected_imu.size() > 0){
    printf("do IMU propagtion + pressure update\n");

    predictor->propagate(state, selected_imu);

    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    assert(!state->foundSPD());

    // get the begin pressure value
    double pres_init = state->getPressureInit();
    double pres_curr = selected_pres.p;

    // update
    updater->updatePressure(state, pres_init, pres_curr, true);

    // is_odom = true;
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


std::vector<std::shared_ptr<Feature>> MsckfManager::selectFeatures(const double time_curr) {

  // ------------------------------- Grab features ------------------------------------------ //

  // grab lost features 
  std::vector<std::shared_ptr<Feature>> feature_lost, feature_marg;
  feature_lost = tracker->get_feature_database()->features_not_containing_newer(time_curr, false, true);

  // grab marginalized features
  if(state->getEstimationNum(CLONE_CAM0) > params.msckf.max_clone_C) {
    double time_marg = state->getMarginalizedTime(CLONE_CAM0);
    feature_marg = tracker->get_feature_database()->features_containing(time_marg, false, true);
  }

  // We also need to make sure that the max tracks does not contain any lost features
  // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
  auto it1 = feature_lost.begin();
  while (it1 != feature_lost.end()) {
    if (std::find(feature_marg.begin(), feature_marg.end(), (*it1)) != feature_marg.end()) {
      it1 = feature_marg.erase(it1);
    } else {
      it1++;
    }
  }

  // ------------------------------- Select features ------------------------------------------ //

  // Concatenate our MSCKF feature arrays: feature lost + feature marginalized 
  std::vector<std::shared_ptr<Feature>> feature_MSCKF = feature_lost;
  feature_MSCKF.insert(feature_MSCKF.end(), feature_marg.begin(), feature_marg.end());

  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector, while ones at the end are long-tracks
  std::sort(feature_MSCKF.begin(), feature_MSCKF.end(), [](const std::shared_ptr<Feature> &a, const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  });                     

  // select the longest tracked feature if in limited computational resources device
  if ((int)feature_MSCKF.size() > params.msckf.max_msckf_update){
    printf("Manager warning: too many features for update, deleted %d\n", 
        feature_MSCKF.size() - params.msckf.max_msckf_update);
        
    feature_MSCKF.erase(feature_MSCKF.begin(), feature_MSCKF.end() - params.msckf.max_msckf_update);
  }

  return feature_MSCKF;
}

} // namespace msckf_dvio
