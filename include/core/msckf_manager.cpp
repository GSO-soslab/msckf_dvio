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
                                                     params.imu_delta_var_1, params.imu_delta_var_2, 
                                                     params.imu_delta, params.dvl_delta, params.gravity,
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

  // double delta_t = data.time - 1614971111.675969;
  // if(delta_t>11 && delta_t<13)
  //   printf("t:%f, a:%f,%f,%f, w:%f,%f,%f\n", data.time, 
  //                                            data.a.x(), data.a.y(),data.a.z(),
  //                                            data.w.x(), data.w.y(),data.w.z());


  //TODO: delete imu messages that are older then some time, like 10s
  //      in case 
}

void MsckfManager::feedDvl(const DvlMsg &data) {
  //// map DVL timestamp because serial driver with 115200 has issue when current profile is set to maximum
  if(mapDvlTime(data)) {
    
    //// append to the buffer 
    buffer_mutex.lock();
    buffer_dvl.insert(buffer_dvl.end(), remapped_queue.begin(), remapped_queue.end());
    buffer_mutex.unlock();

  //// if imu not initialized, feed to initializer
  if(!imu_initializer->isInitialized())
    imu_initializer->feedDvl(remapped_queue); 
  }

  // double delta_t = data.time - 1614971111.675969;
  // if(delta_t>21 )
  //   printf("t:%f, v:%f,%f,%f\n", data.time, data.v.x(),data.v.y(),data.v.z());


  // // filter with timestamp 0.2s
  // // preprocess dvl: remove spikes noise (Median filtering) and data smoothing (Moving Average Filter)
}

void MsckfManager::feedCamera(const ImageMsg &data) {
  buffer_mutex.unlock();
  buffer_img.emplace_back(data);
  buffer_mutex.unlock();

  // tracking feature

}

void MsckfManager::backend() {

  /***** Check Initialization *****/
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


  /***** ? *****/

}
  

bool MsckfManager::mapDvlTime(const DvlMsg &in) {
  /***** re-map DVL timestmaps because of decoding larger current profile data in Serial driver *****/
  /***** BT(0.25), BT(0.5), CP(0.75), BT(1.0) *****/
  bool flag = false;

  //// get integral and fractional part of time
  double integral, fractional; 
  fractional = std::modf(in.time, &integral);

  //// received data in 1 second, now remap
  if(integral != last_integral && last_integral !=0.0) {
    //// clear last remapped data
    remapped_queue.clear();

    // // TEST
    // file.open(file_path, std::ios_base::app);//std::ios_base::app
    // file << std::setprecision(19);

    switch(remap_queue.size()) {
      //// take last 4 
      case 7: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.00, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.25, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(5)) + 0.50, std::get<0>(remap_queue.at(5)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(6)) + 0.75, std::get<0>(remap_queue.at(6)), Eigen::Vector3d(0,0,0));

        last_flag = "#7";
        break;
      }
      
      case 6: {

        if(last_flag == "#2") {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) - 0.50, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) - 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.00, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.25, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.50, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(5)) + 0.75, std::get<0>(remap_queue.at(5)), Eigen::Vector3d(0,0,0));
        }
        else {
          //// take last 4
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.00, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.25, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.50, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(5)) + 0.75, std::get<0>(remap_queue.at(5)), Eigen::Vector3d(0,0,0));
        }

        last_flag = "#6";
        break;
      }
      
      //// take last 4
      case 5: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.00, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.25, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.50, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.75, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));

        last_flag = "#5";
        break;
      }

      //// 1 second has 4 bottom track velocity, v1,v2,v3,v4, assign with 0.25,0.5,0.75,1.0 as fractional part
      //// some case: v4_, v1,v2,v3, we also treat this above just keep code easier
      case 4: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.50, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.75, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));

        last_flag = "#4";
        break;
      }

      //// 1 second has 3 bottom track velocity, lost one measurement during serial transmission
      case 3: {
        //// get fractional
        double v1 = std::get<2>(remap_queue.at(0));
        double v2 = std::get<2>(remap_queue.at(1));
        double v3 = std::get<2>(remap_queue.at(2));
        double avg_1 = (v1+v2)*0.5;
        double avg_2 = (v2+v3)*0.5;

        //// missing one from first set (v1)?, (v2,v3)
        if(avg_1<0.75 && avg_2>0.75) {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.25, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.50, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.75, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        }
        //// missing one from second set (v1,v2), (v3)?
        else if(avg_1<0.5 && avg_2>0.5) {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.50, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        }
        else {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.50, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        }
        
        last_flag = "#3";
        break;
      }

      case 2: {
        // get fractional
        double v1 = std::get<2>(remap_queue.at(0));
        double v2 = std::get<2>(remap_queue.at(1));
        double avg = (v1+v2)*0.5;

        remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
        
        last_flag = "#2";
        break;
      }

      case 1: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));

        last_flag = "#1";
        break;
      }

      //// some other cases
      default : {
        last_flag = "#";
        break;
      }
    }

    // file.close();

    remap_queue.clear();
    flag = true;
  }

  remap_queue.emplace_back(in.v, integral, fractional);
  last_integral = integral;

  // // TEST
  // printf("t:%f\n", in.time);

  return flag;
}

} // namespace msckf_dvio
