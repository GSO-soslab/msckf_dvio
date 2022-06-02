#include "manager/msckf_manager.h"


namespace msckf_dvio
{

MsckfManager::MsckfManager(Params &parameters) : is_odom(false)
{
  // init variable
  this->params = parameters;

  state = std::make_shared<State>(params);

  //// setup imu initializer
  imu_initializer = std::make_shared<ImuInitializer>(params.init, params.prior_imu,
                                                     state->getEstimationValue(DVL, EST_QUATERNION),
                                                     state->getEstimationValue(DVL, EST_POSITION));
  // setup predictor
  predictor = std::make_shared<Predictor>(params.prior_imu);

  // setup updater
  updater = std::make_shared<Updater>(params.prior_dvl, params.msckf);
}

void MsckfManager::feedImu(const ImuMsg &data) {
  //// append to the buffer 
  buffer_mutex.lock();
  buffer_imu.emplace_back(data);
  buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  if(!imu_initializer->isInitialized())
    imu_initializer->feedImu(data);

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
  if(!imu_initializer->isInitialized())
    imu_initializer->feedDvl(data);
}

void MsckfManager::feedCamera(const ImageMsg &data) {
  buffer_mutex.unlock();
  buffer_img.emplace_back(data);
  buffer_mutex.unlock();

  // tracking feature

}

void MsckfManager::backend() {

/******************** Check Initialization ********************/
  if(!imu_initializer->isInitialized()) {
    
    imu_initializer->checkInitialization();

    if(imu_initializer->isInitialized()){

      //// system initialized, get init result
      double time_I0;
      double time_D0;
      Eigen::Vector4d q_I_G0;
      Eigen::Vector3d v_I0;
      Eigen::Vector3d b_a0;
      Eigen::Vector3d b_g0;  
      double time_I_D0;
      std::tie(time_I0, q_I_G0, v_I0, b_g0, b_a0, time_D0, time_I_D0) = imu_initializer->getInitResult();

      //// update state
      Eigen::Matrix<double, 16, 1> imu_value;
      imu_value.block(0, 0, 4, 1) = q_I_G0;
      imu_value.block(4, 0, 3, 1) << 0, 0, 0;
      imu_value.block(7, 0, 3, 1) = v_I0;
      imu_value.block(10, 0, 3, 1) = b_g0; 
      imu_value.block(13, 0, 3, 1) = b_a0;

      state->setTimestamp(time_I0);
      state->setImuValue(imu_value);
      if(params.msckf.do_time_I_D)
        state->setDvlEst(EST_TIMEOFFSET, Eigen::MatrixXd::Constant(1,1,time_I_D0));

      //// clean manager data buffer before initialization 
      buffer_mutex.lock();
      
      auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                    [&](const auto& imu){return imu.time == time_I0 ;});
      buffer_imu.erase(buffer_imu.begin(), frame_imu);

      //! TODO: better dealing with DVL time is same as IMU after initialization
      //!       because most of case, they will not in this case 

      //// also remove DVL that time as same as IMU, that's why +1 
      auto frame_dvl = std::find_if(buffer_dvl.begin(), buffer_dvl.end(),
                    [&](const auto& dvl){return dvl.time > time_D0 ;});
      buffer_dvl.erase(buffer_dvl.begin(), frame_dvl+1);

      //! TODO: clean tracked features before initialization

      buffer_mutex.unlock();

    }
    else
      //// system not initialized, return 
      return;
  }


  //// DO DVL 

  bool new_dvl = checkNewDvl();
  if(new_dvl) {

    /******************** select data from buffer ********************/
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
      printf("Manger error: new sensor measurement time:%f is eariler then current state time:%f\n",
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

      // std::cout<<"state: "<<state->getImuValue()<<std::endl;
      // std::cout<<"cov: "<< state->getCov()<<std::endl;

      test++;
      std::ofstream file;


      predictor->propagate(state, selected_imu);

      file.open(file_path, std::ios_base::app);//std::ios_base::app
      file <<"\n ==================================  " << test<< "  ====================================";
      file<<"\nprop cov: \n" << state->getCov()<<"\n";
      file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
      file.close();

      // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
      assert(!state->foundSPD());

      // std::cout<<"prop state : "<<state->getImuValue()<<std::endl;
      // std::cout<<"prop cov : "<< state->getCov()<<std::endl;

      
      // Last angular velocity (used for cloning when estimating time offset)
      Eigen::Vector3d last_w_I = selected_imu.back().w - state->getEstimationValue(IMU, EST_BIAS_G);
      predictor->augmentDvl(state, last_w_I);

      file.open(file_path, std::ios_base::app);//std::ios_base::app
      file<<"\naug cov: \n" << state->getCov()<<"\n";
      file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
      file.close();

      // update
      Eigen::Vector3d last_v_D = selected_dvl.v;
      updater->updateDvl(state, last_w_I, last_v_D);

      file.open(file_path, std::ios_base::app);//std::ios_base::app
      file<<"\nupdated cov: \n" << state->getCov()<<"\n";
      file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
      file.close();


      // if(test==2){
      //   std::exit(EXIT_FAILURE);
      // }

      // marginalize 

      std::cout<<"clones: "<<state->getEstimationNum(CLONE_DVL)<<std::endl;

      // if max clone of DVL is reached, then do the marginalize: remove the clone and related covarinace
      if(state->getEstimationNum(CLONE_DVL) == params.msckf.max_clone_D) {
        updater->marginalizeDvl(state);
      }

      file.open(file_path, std::ios_base::app);//std::ios_base::app
      file<<"\nmarg cov: \n" << state->getCov()<<"\n";
      file<<"size: "<<state->getCov().rows()<<"X"<<state->getCov().cols()<<"\n";
      file<<"\nupdated IMU: \n" << state->getImuValue().transpose() <<"\n";
      file.close();

      is_odom = true;
    }

  }



}

bool MsckfManager::checkNewDvl() {
  bool new_dvl; 

  buffer_mutex.lock();

  new_dvl = buffer_dvl.size() > 0 ? true : false;  

  buffer_mutex.unlock();

  return new_dvl;
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