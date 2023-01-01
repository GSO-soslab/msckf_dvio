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

  if(buffer_imu.back().time >= param_init.setting.sensor_param[Sensor::IMU].time &&
     buffer_dvl.back().time >= param_init.setting.sensor_param[Sensor::DVL].time &&
     buffer_pressure.back().time >= param_init.setting.sensor_param[Sensor::PRESSURE].time) {

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


  std::cout<<"\n+++++++++++++++++++++++++++++++++++++++\n";                              
  std::cout<<"Initialization result: \n";

  // loop each sensor's init parameter                             
  for(const auto& [sensor, param] : params.init.setting.sensor_param) {

    // stup for each sensor
    switch(sensor) {
      case Sensor::IMU: {

        // convert to Eigen form
        assert(param.state.size() == 16);
        Eigen::Matrix<double, 16, 1> imu_state;
        for(size_t i = 0; i < param.state.size(); i++) {
          imu_state.segment(i,1) = Eigen::Matrix<double,1,1>(param.state.at(i));
        }

        // update to state
        state->setTimestamp(param.time);
        state->setImuValue(imu_state);

        // add to time
        data_time[Sensor::IMU] = param.time;

        // print
        std::cout<<std::fixed <<std::setprecision(9);
        std::cout<<"  IMU:\n";
        std::cout<<"    time: "<< param.time <<"\n";
        std::cout<<std::fixed <<std::setprecision(6);
        std::cout<<"    state: " << imu_state.transpose() << "\n";
        break;
      }

      case Sensor::DVL: {

        // update: 
        // if do online calibration, update to state, otherwise direct update to parameters
        if(params.msckf.do_time_I_D)
          state->setEstimationValue(DVL, EST_TIMEOFFSET, Eigen::MatrixXd::Constant(1,1,param.temporal.at(0)));
        else
          params.prior_dvl.timeoffset = param.temporal.at(0);

        // add to time
        data_time[Sensor::DVL] = param.time;

        // print
        std::cout<<std::fixed <<std::setprecision(9);
        std::cout<<"  DVL: \n";
        std::cout<<"    timestamp: "<< param.time <<"\n";
        std::cout<<"    temporal: "<< param.temporal.at(0) <<"\n";
        break;
      }

      case Sensor::PRESSURE: {

        // update: pressure global right now only has 1 parameter
        state->setPressureInit(param.global.at(0));  

        // add to time
        data_time[Sensor::PRESSURE] = param.time;

        // print
        std::cout<<std::fixed <<std::setprecision(9);
        std::cout<<"  PRESSURE: \n";
        std::cout<<"    timestamp: "<< param.time <<"\n";
        std::cout<<"    global: " << param.global.at(0) <<"\n";
        break;
      }

      default:
        break;
    }
  }
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