#include "core/imu_initializer.h"

namespace msckf_dvio {

void ImuInitializer::feedImu(const ImuMsg &data) {
  buffer_mutex.lock();
  buffer_imu.emplace_back(data);
  buffer_mutex.unlock();
}

void ImuInitializer::feedDvl(const DvlMsg &data) {
  buffer_mutex.lock();
  buffer_dvl.emplace_back(data);
  buffer_mutex.unlock();
}

void ImuInitializer::feedDvl(const std::vector<DvlMsg> &data) {
  buffer_mutex.lock();
  buffer_dvl.insert(buffer_dvl.end(), data.begin(), data.end());
  buffer_mutex.unlock();
}

void ImuInitializer::checkInitialization() {

  //// try to find IMU align time (vehicle suddenly move)
  if(align_time_imu == -1)
    findAlignmentImu();

  //// try to find DVL align time (vehicle suddenly move)
  if(align_time_dvl == -1)
    findAlignmentDvl();

  //// 
  if(align_time_imu != -1 && align_time_dvl != -1 ) {

    std::vector<DvlMsg> dvl_acce;
    std::vector<ImuMsg> imu_acce;
    std::vector<ImuMsg> imu_gyro;

    bool is_ready = grabInitializationData(dvl_acce, imu_acce, imu_gyro);

    if(is_ready)
      doInitialization(dvl_acce, imu_acce, imu_gyro);
  }

}

void ImuInitializer::findAlignmentImu() {

  /*** Select new IMU data for every window size ***/
  std::vector<ImuMsg> selected_imu;

  buffer_mutex.lock();

  if(buffer_imu.end() - buffer_imu.begin() - last_index_imu >= window_imu) {
    std::copy(buffer_imu.begin() + last_index_imu, buffer_imu.end(), std::back_inserter(selected_imu)); 
    last_index_imu = buffer_imu.end() - buffer_imu.begin(); 
  } 

  buffer_mutex.unlock();

  /*** Calculate variance ***/
  if(selected_imu.size() > 0) {
    //// get average of IMU acceleration
    Eigen::Vector3d average = Eigen::Vector3d::Zero();
    for(const auto &imu : selected_imu)
      average += imu.a;
    average /= (int)selected_imu.size();

    //// get variance of IMU acceleration
    double variance = 0.0;
    for (const auto &imu : selected_imu) 
      variance += (imu.a - average).dot(imu.a - average);
    variance = std::sqrt(variance / ((int)selected_imu.size() - 1));
    // printf("IMU Initializer: monitoring jump with variance(%f)\n", variance);

    //// store the latest 3 sections 
    sections_imu.emplace_back(selected_imu, variance);
  }

  /*** Find align section and align point that take suddenly move***/
  if(sections_imu.size() == 3) {

    double var1 = std::get<1>(sections_imu.at(0));
    double var2 = std::get<1>(sections_imu.at(1));
    double var3 = std::get<1>(sections_imu.at(2));
    // // TEST:
    // printf("t:%f~%f, variance:%f, d1:%f, d2:%f\n", std::get<0>(sections_imu.at(2)).front().time, 
    //                                                std::get<0>(sections_imu.at(2)).back().time, 
    //                                                var3, var2-var1, var3-var1);

    //// use some conditions to determine vehcile suddenly movement section
    if((var2-var1 > imu_delta_var_1) && (var3-var1 > imu_delta_var_2)) {
      //// only use the first 2 section, usually the movement in second section, sometimes may in first section
      std::vector<ImuMsg> imu_data;
      imu_data.insert(imu_data.end(), std::get<0>(sections_imu.at(0)).begin(), std::get<0>(sections_imu.at(0)).end());
      imu_data.insert(imu_data.end(), std::get<0>(sections_imu.at(1)).begin(), std::get<0>(sections_imu.at(1)).end());

      //// use delta to determine vehcile suddenly movement
      for(int i=1; i<imu_data.size(); i++) {
        double delta = imu_data.at(i).a.x()-imu_data.at(i-1).a.x();
        //// align point is the last position that vehicle still static, right before vehcile moving 
        if(abs(delta) > imu_delta) {
          align_time_imu = imu_data.at(i-1).time;
          printf("IMU Initializer: find IMU align point at time:%f\n", align_time_imu);
          break;
        }
      }
      //// use 3points to find slope that detemine suddenly movement
      //// Ax=b
      ////  x=(A^T A)^(-1) A^T b
      // Eigen::Matrix<double, 3, 2> A;
      // A << 1,1,2,1,3,1;
      // auto temp1 = A.transpose()*A;
      // auto temp2 =temp1.inverse() * A.transpose();
      // for(int i=2; i<imu_data.size(); i++) {
      //   Eigen::Matrix<double, 3, 1> b;
      //   b(0,0) = imu_data.at(i-2).a.x();
      //   b(1,0) = imu_data.at(i-1).a.x();
      //   b(2,0) = imu_data.at(i).a.x();
      //   Eigen::Matrix<double, 2, 1> line = temp2 * b;
      //   if(abs(line(0,0)) > 0.05) {
      //     align_time_imu = imu_data.at(i).time;
      //     break;
      //   }
      // }

      // // TEST:
      // for(int i=1; i<imu_data.size(); i++)
      // printf("t: %f, a:%f, delta:%f\n",imu_data.at(i).time,  imu_data.at(i).a.x(), imu_data.at(i).a.x()-imu_data.at(i-1).a.x());

    }
    else {
      printf("IMU Initializer: vehicle is not moving \n");
    }

    sections_imu.erase(sections_imu.begin());
  }
}

void ImuInitializer::findAlignmentDvl() {
/*** Select new DVL data for every window size ***/
  buffer_mutex.lock();

  if(buffer_dvl.end() - buffer_dvl.begin() - last_index_dvl >= window_dvl) {
    std::copy(buffer_dvl.begin() + last_index_dvl, buffer_dvl.end(), std::back_inserter(sections_dvl)); 
    last_index_dvl = buffer_dvl.end() - buffer_dvl.begin(); 
  } 

  buffer_mutex.unlock();

/*** Find the point that vehicle suddenly move ***/
  if(sections_dvl.size() >0) {

    for(int i=1; i<sections_dvl.size(); i++) {
      double delta = abs(sections_dvl.at(i).v.x()- sections_dvl.at(i-1).v.x());

      if(delta > dvl_delta) {
        align_time_dvl = sections_dvl.at(i-1).time;
        printf("IMU Initializer: find DVL align point at time:%f\n", align_time_dvl);
        break;
      }

      // TEST:
      // printf("t:%f, v:%f, d:%f\n", sections_dvl.at(i).time, sections_dvl.at(i).v.x(), 
      //                              sections_dvl.at(i).v.x()- sections_dvl.at(i-1).v.x());
    }

    //// if no jump point found, delete all but the last one
    sections_dvl.erase(sections_dvl.begin(), sections_dvl.end()-1);
  }
}

bool ImuInitializer::grabInitializationData(std::vector<DvlMsg> &dvl_a,
                                            std::vector<ImuMsg> &imu_a, 
                                            std::vector<ImuMsg> &imu_g) {
  //// TODO: erase the data before alignment 
  bool ready = false;

  /*** Check if DVL velocity section that always increase is found ***/
  /*** grabbed velocity example in x-axis: ... 0, [0, 0.1, 0.2, 0.3, 0.4,] 0.35, ... ***/
  std::vector<DvlMsg> selected_dvl;
  buffer_mutex.lock();

  //// used align_time_dvl find the corresponding index in the buffer
  auto selected_frame = std::find_if(buffer_dvl.begin(), buffer_dvl.end(),
                        [&](const auto& dvl){return dvl.time == align_time_dvl;});
  std::copy(selected_frame, buffer_dvl.end(), std::back_inserter(selected_dvl)); 
  
  buffer_mutex.unlock();

  //// check if velocity start to decrease
  int index;
  for(index=1; index<selected_dvl.size(); index++) {
    if(abs(selected_dvl.at(index).v.x()) < abs(selected_dvl.at(index-1).v.x())) {
      ready = true;
      break;
    }
  }

  //// if not found, just return
  if(!ready)  {return false;}

  /***** Get IMU data for accleration initialization *****/

  //// grab DVL section that has increaseing velocity
  selected_dvl.erase(selected_dvl.begin() + index, selected_dvl.end());
  // TEST:
  // for(const auto &dvl: selected_dvl)
  //   printf("t: %f, vx: %f\n",dvl.time,dvl.v.x());

  //// get time offset between IMU and DVL
  double time_offset = align_time_imu - align_time_dvl; //// −10.645267
  double last_dvl_time = selected_dvl.back().time + time_offset;
  
  buffer_mutex.lock();

  //// select cooresponding IMU, select IMUs until last one timestamp > DVL timestamp
  auto frame_end = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                   [&](const auto& imu){return imu.time > last_dvl_time ;});

  if(frame_end != buffer_imu.end())
    std::copy(buffer_imu.begin(), frame_end + 1, std::back_inserter(imu_a));
  else
    //// DVL data is arrived eariler then IMU, so return and waitting
    return false;

  buffer_mutex.unlock();

  //// find IMU at alignment point as begining
  auto frame_beg = std::find_if(imu_a.begin(), imu_a.end(),
                   [&](const auto& imu){return imu.time == align_time_imu;});

  //// earse all the data before align point
  imu_a.erase(imu_a.begin(), frame_beg);
  // TEST:
  // for(const auto &imu: imu_a) 
  //   printf("t: %f\n", imu.time);

/*** Get DVL data for accleration initialization ***/
  linearInterp(time_offset, imu_a, selected_dvl, dvl_a);
  // TEST:
  // for(const auto dvl:dvl_a)
  //   std::cout<<std::setprecision(17)<<"t: "<<dvl.time<<std::setprecision(7)<<
  //     " v: "<< dvl.v.x()<<" "<< dvl.v.y()<<" "<< dvl.v.z()<<
  //     " a: "<< dvl.a.x()<<" "<< dvl.a.y()<<" "<< dvl.a.z()<<"\n";


/*** Get IMU data for gyro initialization ***/

  //// grab IMU section that before suddenly movement point
  //// assuming gyro data before that point is similar like stationary
  buffer_mutex.lock();

  auto frame_gyro = std::find_if(buffer_imu.begin(), buffer_imu.end(),
                    [&](const auto& imu){return imu.time == align_time_imu ;});
  std::copy(frame_gyro - 100, frame_gyro, std::back_inserter(imu_g));

  buffer_mutex.unlock();
  // TEST:
  // for(const auto &imu : imu_g)
  //   printf("t: %f\n", imu.time);

  return ready;
}

void ImuInitializer::linearInterp(const double time_offset, const std::vector<ImuMsg> &imu_in,
                                  const std::vector<DvlMsg> &dvl_in, std::vector<DvlMsg> &dvl_out) {
/***** Pre-processing: align DVL and IMU timestamp; calculate DVL acceleration *****/

  //// calculate DVL acceleration( v_(t) - v_(t-1) / delta_t), and subtract time_base, and copy velocity
  std::vector<DvlMsg> dvl_data; 

  dvl_data.emplace_back(0.0, dvl_in.front().v, Eigen::Vector3d(0,0,0));
  for (int i=1; i<dvl_in.size(); i++) {
    Eigen::Vector3d accel = (dvl_in.at(i).v    - dvl_in.at(i-1).v   ) / 
                            (dvl_in.at(i).time - dvl_in.at(i-1).time);
    //// make the align point as time origin
    dvl_data.emplace_back(dvl_in.at(i).time + time_offset, dvl_in.at(i).v, accel);
  }

/***** Interpolate DVL with IMU timestep *****/
  //// select every two measurement
  int selected_index=0;
  for(int i=0; i<dvl_data.size()-1; i++) {
    //// second DVL frame time
    double t2 = dvl_data.at(i+1).time;

    //// select IMU inside two DVL frame
    std::vector<ImuMsg> selected_imu;
    auto selected_frame = std::find_if(imu_in.begin() + selected_index, imu_in.end(), 
                          [&](const auto& imu){return imu.time > t2 ;});
    std::copy(imu_in.begin() + selected_index, selected_frame, std::back_inserter(selected_imu));
    selected_index = selected_frame - imu_in.begin();

    //// interpolate acceleration: f(t) = (f(t)2-f(t)1) / (t2-t1) * (t-t1) + f(t)1
    for(const auto &imu : selected_imu) {
      //// interpolate acceleration in x-axis
      double a_x = (dvl_data.at(i+1).a.x() - dvl_data.at(i).a.x()) /
                   (dvl_data.at(i+1).time - dvl_data.at(i).time) *
                   (imu.time - dvl_data.at(i).time) + dvl_data.at(i).a.x();
      //// interpolate acceleration in y-axis
      double a_y = (dvl_data.at(i+1).a.y() - dvl_data.at(i).a.y()) /
                   (dvl_data.at(i+1).time - dvl_data.at(i).time) *
                   (imu.time - dvl_data.at(i).time)  + dvl_data.at(i).a.y();
      //// interpolate acceleration in z-axis
      double a_z = (dvl_data.at(i+1).a.z() - dvl_data.at(i).a.z()) /
                   (dvl_data.at(i+1).time - dvl_data.at(i).time) *
                   (imu.time - dvl_data.at(i).time) + dvl_data.at(i).a.z();
      //// interplate velocity in x-axis
      double v_x = (dvl_data.at(i+1).v.x() - dvl_data.at(i).v.x()) /
                   (dvl_data.at(i+1).time - dvl_data.at(i).time) *
                   (imu.time - dvl_data.at(i).time) + dvl_data.at(i).v.x();
      //// interpolate acceleration in y-axis
      double v_y = (dvl_data.at(i+1).v.y() - dvl_data.at(i).v.y()) /
                   (dvl_data.at(i+1).time - dvl_data.at(i).time) *
                   (imu.time - dvl_data.at(i).time)  + dvl_data.at(i).v.y();
      //// interpolate acceleration in z-axis
      double v_z = (dvl_data.at(i+1).v.z() - dvl_data.at(i).v.z()) /
                   (dvl_data.at(i+1).time - dvl_data.at(i).time) *
                   (imu.time - dvl_data.at(i).time) + dvl_data.at(i).v.z();

      //// store interpolated data
      dvl_out.emplace_back(imu.time - time_offset, Eigen::Vector3d(v_x,v_y,v_z), Eigen::Vector3d(a_x,a_y,a_z));
    }
  }

  //// imu_in will have one more data because that data timestamp larger then DVL timestamp
  assert(dvl_out.size() == imu_in.size() - 1);
  
}

      
void ImuInitializer::doInitialization(const std::vector<DvlMsg> &dvl_a, 
                                      const std::vector<ImuMsg> &imu_a,
                                      const std::vector<ImuMsg> &imu_g) {
  //// TODO: change this as a calibration parameters or state parameters
  //// TODO: set manually set bias
  
  Eigen::Matrix3d R_I_D;
  Eigen::Vector3d p_I_D;

  R_I_D = T_I_D.block<3,3>(0,0);
  p_I_D = T_I_D.block<3,1>(0,3);

  //// transformation between Gloabl inertial frame and IMU local frame
  Eigen::Matrix3d R_I_G;

/*** Acceleration bias and gravity projection estimation ***/
  Eigen::Vector3d ba_avg;
  //// first data is not using since the vehicle is assuming not moving
  for(int i=1; i<dvl_a.size(); i++) {
    //// Estimate DVL measured acceleration in IMU frame: Troni and Whitcomb JFR 2015, Eq.12
    //// a_I_hat = [w_I]x R_I_D * v_D + R_I_D * a_D - ([w_I]x^2 + [w_I_dot]x) * p_I_D
    auto skew = toSkewSymmetric(imu_a.at(i).w);
    auto skew_sq = skew * skew;
    //// calculate derivative of angular velocity: w_a_dot, then do skew symetric
    auto skew_dot = msckf_dvio::toSkewSymmetric( 
                    (imu_a.at(i).w - imu_a.at(i-1).w) /
                    (imu_a.at(i).time - imu_a.at(i-1).time));

    auto a_I_hat = skew * R_I_D * dvl_a.at(i).v  + R_I_D * dvl_a.at(i).a - (skew_sq + skew_dot) * p_I_D;
    //// compensate IMU acceleration by DVL measurement
    auto a_compensated = imu_a.at(i).a - a_I_hat;

    //// Construct a frame that Z-axis aline with IMU measurement(0,0,-9.81) 

    //// get rotation of Z part: cosine of z with respect to reference frame's x,y,z axis (normalization)
    Eigen::Vector3d z_I_G = a_compensated / a_compensated.norm();
    
    //// construct x-axis to perpendicular to z-axis
    //// Use [Gram-Schmidt Process](https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
    Eigen::Vector3d x_I(1, 0, 0);
    Eigen::Vector3d x_I_G = x_I - z_I_G * z_I_G.transpose() * x_I;
    //// get rotation of X part
    x_I_G = x_I_G / x_I_G.norm();

    //// construct y-axis from the cross product of these two
    //// get rotation of Y part
    Eigen::Vector3d y_I_G = msckf_dvio::toSkewSymmetric(z_I_G) * x_I_G;

    // construct rotation matrix
    R_I_G.block(0, 0, 3, 1) = x_I_G;
    R_I_G.block(0, 1, 3, 1) = y_I_G;
    R_I_G.block(0, 2, 3, 1) = z_I_G;

    /// bias = measurement - prjected_gravity
    Eigen::Vector3d ba = a_compensated - R_I_G * Eigen::Vector3d(0.0, 0.0, gravity);
    ba_avg += ba;
  }
  //// average the bias estimation
  ba_avg = ba_avg / (dvl_a.size()-1);

/*** Gyro bias estimation ***/
  Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();

  for(const auto &imu : imu_g) 
    avg_gyro += imu.w;
  
  avg_gyro /= imu_g.size();
  
  // TESTs:
  printf("Initialization result at DVL time:%f\n, ba:%f,%f,%f\n bg:%f,%f,%f\n R_I_G:%f,%f,%f\n%f,%f,%f\n,%f,%f,%f\n", 
         dvl_a.back().time, 
         ba_avg.x(),ba_avg.y(),ba_avg.z(),
         avg_gyro.x(),avg_gyro.y(),avg_gyro.z(),
         R_I_G(0,0),R_I_G(0,1),R_I_G(0,2),
         R_I_G(1,0),R_I_G(1,1),R_I_G(1,2),
         R_I_G(2,0),R_I_G(2,1),R_I_G(2,2));

  is_initialized = true;
}


}
