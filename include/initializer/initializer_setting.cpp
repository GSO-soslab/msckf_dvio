#include "initializer_setting.h"

using namespace msckf_dvio;

InitSetting::InitSetting(paramInit param_init) 
    : Initializer(param_init) {

  // setup the sensors used for this system initialization
  for(const auto& [sensor, param] : param_init.setting) {
    sensors.push_back(sensor);
  }

}

bool InitSetting::useSensor(const Sensor &sensor) {
  if(std::find(sensors.begin(), sensors.end(), sensor) != sensors.end()){
    return true;
  }
  else {
    return false;
  }
}

void InitSetting::checkInit() {
  std::unique_lock<std::recursive_mutex> lck(buffer_mutex);

  int count = 0;

  for(const auto& sensor : sensors) {
    // check all data buffer are available
    switch (sensor)
    {
      case Sensor::IMU: {
        if(buffer_imu.size() > 0 && 
           buffer_imu.back().time >= param_init.setting[Sensor::IMU].time) {
          count ++;
        }
        break;
      }

      case Sensor::DVL: {
        if(buffer_dvl.size() > 0 && 
           buffer_dvl.back().time >= param_init.setting[Sensor::DVL].time) {
          count ++;
        }
        break;
      }

      case Sensor::PRESSURE: {
        if(buffer_pressure.size() > 0 && 
           buffer_pressure.back().time >= param_init.setting[Sensor::PRESSURE].time) {
          count ++;
        }
        break;        
      }

      default:
        break;
    }
  }

  // check if all the sensors are available
  if(count == sensors.size()) {
    initialized = true;

    cleanBuffer();    
  }

}

void InitSetting::updateInit(std::shared_ptr<State> state, 
                             Params &params, 
                             std::map<Sensor, double> &data_time) {


  std::cout<<"\n+++++++++++++++++++++++++++++++++++++++\n";                              
  std::cout<<"Initialization result: \n";

  // loop each sensor's init parameter                             
  for(const auto& [sensor, param] : params.init.setting) {

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
        std::cout<<std::fixed <<std::setprecision(6);
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