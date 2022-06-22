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
                                                      
  // setup predictor
  predictor = std::make_shared<Predictor>(params.prior_imu);

  // setup updater
  updater = std::make_shared<Updater>(params.prior_dvl, params.msckf);

  // setup tracker
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
  buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  if(!initializer->isInit())
    initializer->feedDvl(data);
}

void MsckfManager::feedPressure(const PressureMsg &data) {
  //// append to the buffer 
  buffer_mutex.unlock();
  buffer_pressure.emplace_back(data);
  buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  //! TODO: check if this sensor is need for initialization
  //! initializer->needSensor(PRESSURE) ?
  if(!initializer->isInit())
    initializer->feedPressure(data);
}

void MsckfManager::feedCamera(ImageMsg &data) {
  // append to sensor buffer
  buffer_mutex.unlock();
  buffer_img_time.emplace_back(data.time);
  buffer_mutex.unlock();

  // do tracking
  tracker->feed_monocular(data.time, data.image, 0);

  //// [time, features(lost+marg)]

  //// clean database


  std::shared_ptr<FeatureDatabase> database = tracker->get_feature_database();
  std::vector<std::shared_ptr<Feature>> feats_lost = database->features_not_containing_newer(data.time);

  // delete access features
  for (auto const &f : feats_lost) {
    f->to_delete = true;
  }
  database->cleanup();

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

  if(!initializer->isInit()) {

    initializer->checkInit();

    if(initializer->isInit()){
      std::vector<double> data_time;
      initializer->updateInit(state, params, data_time);

      //// clean manager data buffer before initialization 
      buffer_mutex.lock();
      
      // delete IMU data used for initialization
      auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                    [&](const auto& imu){return imu.time == data_time.at(0) ;});
      if (frame_imu != buffer_imu.end())
        buffer_imu.erase(buffer_imu.begin(), frame_imu);

      // delete DVL BT data used for initialization
      auto frame_dvl = std::find_if(buffer_dvl.begin(), buffer_dvl.end(),
                    [&](const auto& dvl){return dvl.time > data_time.at(1) ;});
      if (frame_dvl != buffer_dvl.end())                    
        buffer_dvl.erase(buffer_dvl.begin(), frame_dvl);

      // delete DVL pressure data used for initialization
      auto frame_pres = std::find_if(buffer_pressure.begin(), buffer_pressure.end(),
                    [&](const auto& pressure){return pressure.time > data_time.at(2) ;});
      if (frame_pres != buffer_pressure.end())                    
        buffer_pressure.erase(buffer_pressure.begin(), frame_pres);

      //! TODO: clean tracked features before initialization

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

  switch(selectUpdateSource()) {

    // choose DVL BT velocity to update IMU
    case VELOCITY_BT: 
      doDVL();
      break;

    // choose DVL CP pressure to update IMU
    case PRESSURE_CP: 
      doPressure();
      break;

    // no sensor data avaiable
    case NONE: 
      break;
  }


}

UpdateSouce MsckfManager::selectUpdateSource() {
  double early_time = std::numeric_limits<double>::infinity();
  UpdateSouce source = NONE;

  buffer_mutex.lock();

  // check DVL BT velocity
  if(buffer_dvl.size() > 0){
    double time = buffer_dvl.front().time;
    if(time < early_time){
      early_time = time;
      source = VELOCITY_BT;
    }
  }

  // check DVL CP pressure
  if(buffer_pressure.size() > 0){
    double time = buffer_pressure.front().time;
    if(time < early_time){
      early_time = time;
      source = PRESSURE_CP;
    }
  }

  buffer_mutex.unlock();

  return source;
}

void MsckfManager::doDVL() {

  /******************** select IMU data for propagation ********************/
  /******************** select DVL data for update ********************/
  DvlMsg selected_dvl;
  std::vector<ImuMsg> selected_imu;

  buffer_mutex.lock();

  /// select first DVL data
  selected_dvl = buffer_dvl.front();

  //// determine selected IMU range, from last updated to current sensor measurement
  auto offset = params.msckf.do_time_I_D ? 
                state->getEstimationValue(DVL, EST_TIMEOFFSET)(0) : params.prior_dvl.timeoffset;
  double time_prev_state = state->getTimestamp();
  double time_curr_state = selected_dvl.time + offset;

  //// make sure sensors data is not eariler then current state
  if(time_curr_state <= time_prev_state) {
    printf("Manger error: new BT measurement time:%f is eariler then current state time:%f\n",
            time_curr_state, time_prev_state);
    std::exit(EXIT_FAILURE);
  }

  //// make sure IMU data asscoiated with sensor is arrived, means IMU time > current sensor time 
  auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                [&](const auto& imu){return imu.time > time_curr_state ;});
  if(frame_imu != buffer_imu.end()) {
    //// make sure we have at least 2 IMU data
    selected_imu = selectImu(time_prev_state, time_curr_state);

    // earse selected DVL measurement
    buffer_dvl.erase(buffer_dvl.begin());
    // earse selected IMU data
    // leave one IMU data time <= current state time, so we can interploate later
    buffer_imu.erase(buffer_imu.begin(), frame_imu-1);      
  }

  buffer_mutex.unlock();

  /******************** do the msckf stuff ********************/

  if(selected_imu.size() > 0){

    // printf("beg=%f, end=%f\n", time_prev_state, time_curr_state);
    // for(const auto &i:selected_imu) {
    //   printf("t=%f\n", i.time);
    // }

    test++;
    std::ofstream file;


    predictor->propagate(state, selected_imu);
    // file.open(file_path, std::ios_base::app);//std::ios_base::app
    // file <<"\n ==================================  " << test<< "  ====================================";
    // file<<"\nprop cov: \n" << state->getCov()<<"\n";
    // file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
    // file.close();

    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    assert(!state->foundSPD());

    // Last angular velocity and linear velocity
    Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
    Eigen::Vector3d last_v_D = selected_dvl.v;

    // // use Last angular velocity for cloning when estimating time offset)
    if(params.msckf.max_clone_D != 0)
      predictor->augmentDvl(state, last_w_I);
    // file.open(file_path, std::ios_base::app);//std::ios_base::app
    // file<<"\naug cov: \n" << state->getCov()<<"\n";
    // file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
    // file.close();

    // update
    // updater->updateDvl(state, last_w_I, last_v_D);
    updater->updateDvl(state, last_w_I, last_v_D, true);
    // file.open(file_path, std::ios_base::app);//std::ios_base::app
    // file<<"\nupdated cov: \n" << state->getCov()<<"\n";
    // file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
    // file.close();


    // if(test==2){
    //   std::exit(EXIT_FAILURE);
    // }

    // marginalize 
    // if max clone of DVL is reached, then do the marginalize: remove the clone and related covarinace
    if(params.msckf.max_clone_D != 0 &&
       params.msckf.max_clone_D == state->getEstimationNum(CLONE_DVL)) {
      updater->marginalizeDvl(state);
    }
    // file.open(file_path, std::ios_base::app);//std::ios_base::app
    // file<<"\nmarg cov: \n" << state->getCov()<<"\n";
    // file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
    // file<<"\nupdated IMU: \n" << state->getImuValue().transpose() <<"\n";
    // file.close();

    is_odom = true;
  }

}

void MsckfManager::doPressure() {

  /******************** select IMU data for propagation ********************/
  /******************** select pressure data for update ********************/

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

    predictor->propagate(state, selected_imu);

    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    assert(!state->foundSPD());

    // get the begin pressure value
    double pres_init = state->getPressureInit();
    double pres_curr = selected_pres.p;

    // update
    updater->updatePressure(state, pres_init, pres_curr, true);

    is_odom = true;
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

} // namespace msckf_dvio
