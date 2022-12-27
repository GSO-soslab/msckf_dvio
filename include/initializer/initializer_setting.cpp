#include "initializer_setting.h"

using namespace msckf_dvio;

InitSetting::InitSetting(paramInit param_init) 
    : Initializer(param_init) {}

void InitSetting::checkInit() {
  std::unique_lock<std::recursive_mutex> lck(buffer_mutex);

  //! TODO: use switch(Sensor::name) to check last buffer element timestamp 

  // check if all buffer has data
  if(buffer_imu.size() == 0 ||
     buffer_dvl.size() == 0 ||
     buffer_pressure.size() == 0) {

    return;
  }

  // check data stream is arrived after sensor initialized timestamp
  if(buffer_imu.back().time >= param_init.setting.time[Sensor::IMU] &&
     buffer_dvl.back().time >= param_init.setting.time[Sensor::DVL] &&
     buffer_pressure.back().time >= param_init.setting.time[Sensor::PRESSURE]) {

    initialized = true;

    cleanBuffer();
  }

}

bool InitSetting::useSensor(const Sensor &sensor) {
  // this initializer will use IMU, DVL, Pressure data

  if(sensor == Sensor::IMU || sensor == Sensor::DVL || sensor == Sensor::PRESSURE) {
    return true;
  }
  else {
    return false;
  }
}

void InitSetting::updateInit(std::shared_ptr<State> state, 
                             Params &params, 
                             std::map<Sensor, double> &data_time) {

  // ====================== get init result ====================== //

  Eigen::Matrix<double, 17, 1> state_imu;
  state_imu.segment(0,1) = Eigen::Matrix<double,1,1>(params.init.setting.time[Sensor::IMU]);
  state_imu.segment(1,4) = params.init.setting.orientation;
  state_imu.segment(5,3) = params.init.setting.position;
  state_imu.segment(8,3) = params.init.setting.velocity;
  state_imu.segment(11,3) = params.init.setting.bias_gyro;
  state_imu.segment(14,3) = params.init.setting.bias_acce;

  Eigen::Matrix<double, 2, 1>  state_dvl;
  state_dvl.segment(0,1) = Eigen::Matrix<double,1,1>(params.init.setting.time[Sensor::DVL]);
  state_dvl.segment(1,1) = Eigen::Matrix<double,1,1>(params.init.setting.temporal[Sensor::DVL]);

  // ====================== update IMU related ====================== //

  state->setTimestamp(state_imu(0));
  state->setImuValue(state_imu.tail(16));

  // ====================== update DVL related ====================== //

  // if do online calibration, update to state, otherwise direct update to parameters
  if(params.msckf.do_time_I_D)
    state->setEstimationValue(DVL, EST_TIMEOFFSET, Eigen::MatrixXd::Constant(1,1,state_dvl(1)));
  else
    params.prior_dvl.timeoffset = state_dvl(1);

  // ====================== update Pressure related ====================== //
  
  state->setPressureInit(params.init.setting.global[Sensor::PRESSURE].at(0));  

  // ====================== return data time to clean buffer ====================== //

  data_time[Sensor::IMU] = params.init.setting.time[Sensor::IMU];
  data_time[Sensor::DVL] = params.init.setting.time[Sensor::DVL];
  data_time[Sensor::PRESSURE] = params.init.setting.time[Sensor::PRESSURE];

  printf("\nInitializer(Setting): done !\n");
}

void InitSetting::cleanBuffer() {
  std::unique_lock<std::recursive_mutex> lck(buffer_mutex);

  // clear base buffer
  std::vector<ImuMsg>().swap(buffer_imu);
  std::vector<DvlMsg>().swap(buffer_dvl);
  std::vector<PressureMsg>().swap(buffer_pressure);

  // clear this initializer buffer
  // do something?
}