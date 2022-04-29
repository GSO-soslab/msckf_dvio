#include "core/msckf_manager.h"


namespace msckf_dvio
{

MsckfManager::MsckfManager(Params &parameters)
{
  // init variable
  this->params = parameters;

  state = std::make_shared<State>(params.timeoffset_I_D, params.dvl_extrinsics, params.scale);

  //// setup imu initializer
  //// TODO: set parameters into sub_param like param_imu_init
  imu_initializer = std::make_shared<ImuInitializer>(params.imu_windows, params.dvl_windows,
                                                     params.imu_var, params.imu_delta, 
                                                     params.dvl_delta, params.gravity,
                                                     state->dvl_extrinsic_->transformation());
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
  //// TODO: preprocess dvl:
  ////  1) correct timeoffset: basic substract time transmit from DVl to bottom, check duraction field of df21
  ////  2) remove spikes noise (Median filtering) and data smoothing (Moving Average Filter)

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
      double time_I_D0;
      double time_I0;
      double time_D0;
      Eigen::Vector4d q_I_G0;
      Eigen::Vector3d v_I0;
      Eigen::Vector3d b_a0;
      Eigen::Vector3d b_g0;  
      std::tie(time_I0, q_I_G0, v_I0, b_g0, b_a0, time_D0, time_I_D0) = imu_initializer->getInitResult();

      //// update state
      Eigen::Matrix<double, 16, 1> imu_value;
      imu_value.block(0, 0, 4, 1) = q_I_G0;
      imu_value.block(4, 0, 3, 1) << 0, 0, 0;
      imu_value.block(7, 0, 3, 1) = v_I0;
      imu_value.block(10, 0, 3, 1) = b_g0; 
      imu_value.block(13, 0, 3, 1) = b_a0;

      state->timestamp_ = time_I0;
      state->imu_->setValue(imu_value);
      state->dvl_timeoffset_ = time_I_D0;

      //// clean manager data buffer before initialization 
      buffer_mutex.lock();
      
      auto frame_imu = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                    [&](const auto& imu){return imu.time == time_I0 ;});
      buffer_imu.erase(buffer_imu.begin(), frame_imu);

      auto frame_dvl = std::find_if(buffer_dvl.begin(), buffer_dvl.end(),
                    [&](const auto& dvl){return dvl.time > time_D0 ;});
      buffer_dvl.erase(buffer_dvl.begin(), frame_dvl);

      buffer_mutex.unlock();

    }
    else
      //// system not initialized, retuen 
      return;
  }


/******************** Propagate and Clone ********************/
  //// determine ready to propagate: IMU time > first DVL data in the buffer or image 
  // time_dvl  = DVL_time.at(0);
  // std::find_if(imu.time > time_dvl )

  // if(find) {
  //   propagator->propagate(selected_imu);
  // }


/******************** Update ********************/

}


} // namespace msckf_dvio
