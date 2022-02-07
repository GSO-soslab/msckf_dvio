#include <vector>
#include <tuple>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <cassert>

#include "types/type.h"
#include "utils/utils.h"

//// TODO: 1) change the vector_tuple STLs into ImuMsg data structure
//// TODO: 2) may not need time after interpolation, becasue they are one-by-one aligened

void linearInterp(const double time_offset, 
                  const std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> &imu_in,
                  const std::vector<std::tuple<double, Eigen::Vector3d>> &dvl_in,
                        std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> &dvl_out) {
  //// select First DVL measurement time as the base timestamp
  double time_base = std::get<0>(dvl_in.at(0)) ;

  //// align IMU time into DVL frame, and make the align point as time origin
  std::vector<double> imu_time;
  for(const auto &imu:imu_in)
    imu_time.push_back(std::get<0>(imu) - time_offset - time_base);

  //// calculate DVL acceleration( v_(t) - v_(t-1) / delta_t), and subtract time_base, and copy velocity
  std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> dvl_data; //// t, accel, velocity

  dvl_data.emplace_back(0.0, Eigen::Vector3d(0,0,0), std::get<1>(dvl_in.front()));
  for (int i=1; i<dvl_in.size(); i++) {
    Eigen::Vector3d accel = (std::get<1>(dvl_in.at(i)) - std::get<1>(dvl_in.at(i-1))) / 
                        (std::get<0>(dvl_in.at(i)) - std::get<0>(dvl_in.at(i-1)));
    dvl_data.emplace_back(std::get<0>(dvl_in.at(i))-time_base, accel, std::get<1>(dvl_in.at(i)) );
  }
  // // TEST
  // for(const auto dvl:dvl_data)
  //   std::cout<<"t: "<<std::get<0>(dvl)<<" v: "<<std::get<1>(dvl).x()
  //            <<" "<<std::get<1>(dvl).y()<<" "<<std::get<1>(dvl).z()<<std::endl;

  //// select every two measurement
  for(int i=0; i<dvl_data.size()-1; i++) {
    // double t1 = std::get<0>(dvl_data.at(i));
    double t2 = std::get<0>(dvl_data.at(i+1));

    //// select IMU inside t1~t2
    std::vector<double> imu_selected;
    auto selected_frame = std::find_if(imu_time.begin(), imu_time.end(), [&](const auto& t){return t > t2 ;});
    std::copy(imu_time.begin(), selected_frame, std::back_inserter(imu_selected));
    imu_time.erase(imu_time.begin(), selected_frame);

    //// interpolate acceleration: f(t) = (f(t)2-f(t)1) / (t2-t1) * (t-t1) + f(t)1
    for(const auto &t:imu_selected) {
      //// interpolate acceleration in x-axis
      double a_x = (std::get<1>(dvl_data.at(i+1)).x() - std::get<1>(dvl_data.at(i)).x()) /
                   (std::get<0>(dvl_data.at(i+1)) - std::get<0>(dvl_data.at(i))) *
                   (t - std::get<0>(dvl_data.at(i))) + std::get<1>(dvl_data.at(i)).x();
      //// interpolate acceleration in y-axis
      double a_y = (std::get<1>(dvl_data.at(i+1)).y() - std::get<1>(dvl_data.at(i)).y()) /
                   (std::get<0>(dvl_data.at(i+1)) - std::get<0>(dvl_data.at(i))) *
                   (t - std::get<0>(dvl_data.at(i)))  + std::get<1>(dvl_data.at(i)).y();
      //// interpolate acceleration in z-axis
      double a_z = (std::get<1>(dvl_data.at(i+1)).z() - std::get<1>(dvl_data.at(i)).z()) /
                   (std::get<0>(dvl_data.at(i+1)) - std::get<0>(dvl_data.at(i))) *
                   (t - std::get<0>(dvl_data.at(i))) + std::get<1>(dvl_data.at(i)).z();

      //// interplate velocity in x-axis
      double v_x = (std::get<2>(dvl_data.at(i+1)).x() - std::get<2>(dvl_data.at(i)).x()) /
                   (std::get<0>(dvl_data.at(i+1)) - std::get<0>(dvl_data.at(i))) *   
                   (t - std::get<0>(dvl_data.at(i))) + std::get<2>(dvl_data.at(i)).x();
      //// interplate velocity in y-axis
      double v_y = (std::get<2>(dvl_data.at(i+1)).y() - std::get<2>(dvl_data.at(i)).y()) /
                   (std::get<0>(dvl_data.at(i+1)) - std::get<0>(dvl_data.at(i))) *   
                   (t - std::get<0>(dvl_data.at(i))) + std::get<2>(dvl_data.at(i)).y();
      //// interplate velocity in z-axis
      double v_z = (std::get<2>(dvl_data.at(i+1)).z() - std::get<2>(dvl_data.at(i)).z()) /
                   (std::get<0>(dvl_data.at(i+1)) - std::get<0>(dvl_data.at(i))) *   
                   (t - std::get<0>(dvl_data.at(i))) + std::get<2>(dvl_data.at(i)).z();

      //// store interpolated data
      dvl_out.emplace_back(time_base+t, Eigen::Vector3d(a_x,a_y,a_z), Eigen::Vector3d(v_x,v_y,v_z));
    }
  }

  //// imu_in will have one more data because that data timestamp larger then DVL timestamp
  assert(dvl_out.size() == imu_in.size() - 1);
  
}

int main() {

  /*** Get DVL data: 1) start: one data before vehicle is moving 
   *                 2) end: one data before velocity decreased (the duration DVL has positive accleration)
   ***/
  std::vector<std::tuple<double, Eigen::Vector3d>> dvl_data;
  //// remap DVL time with standard timestamp: 0.25,0.5,0.75,1.0 ...
  Eigen::Vector3d v1;
  v1<<0.074600, 0.013400,-0.006300;
  dvl_data.emplace_back(1614971134.000,v1);
  Eigen::Vector3d v2;
  v2<<0.164700,-0.074800,0.005200;
  dvl_data.emplace_back(1614971134.250,v2);
  Eigen::Vector3d v3;
  v3<<0.288500,-0.033300,0.013800;
  dvl_data.emplace_back(1614971134.500,v3);
  Eigen::Vector3d v4;
  v4<<0.334100,-0.015000,-0.008300;
  dvl_data.emplace_back(1614971134.750,v4);
  Eigen::Vector3d v5;
  v5<<0.364400,-0.014500,-0.011300;
  dvl_data.emplace_back(1614971135.000,v5);


  /*** Get IMU data: 1) start: one data before vehcile is moving 
   *                 2) end: based on DVL data,but one more data timestamp larger then DVL timestamp
   * ***/
  std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> imu_data;

  std::fstream file;
  file.open("/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/notes/init_align_imu.txt",std::ios::in); 

  if (file.is_open()){  
    std::string line;
    while(std::getline(file, line)){ 
        const char *c = line.c_str();

        double time,a1,a2,a3,w1,w2,w3;
        if(sscanf(c, "t:%lf, a:%lf,%lf,%lf, w:%lf,%lf,%lf", &time,&a1,&a2,&a3, &w1,&w2,&w3) >= 1) {
          Eigen::Vector3d a,w;
          a<<a1,a2,a3;
          w<<w1,w2,w3;
          imu_data.emplace_back(time,a,w);
        }
    }
    file.close();   //close the file object.
  }

  std::cout<<"Finsih read DVL and IMU data!!\n";
  // // TEST:
  // for(const auto &imu:imu_data)
  //   std::cout<<std::setprecision(17)<<"t: "<<std::get<0>(imu)
  //            <<std::setprecision(8)<<" a: "<<std::get<1>(imu).x()<<","<<std::get<1>(imu).y()<<","<<std::get<1>(imu).z() 
  //            <<" w: "<<std::get<2>(imu).x()<<","<<std::get<2>(imu).y()<<","<<std::get<2>(imu).z()<< std::endl;

  /*** Align IMU and DVL ***/

  //// Align IMU and DVL : first time are selected as one before IMU and DVL measurement start tp jump
  //// t_imu = t_dvl + t_offset
  double t_offset = 1614971123.354733-1614971134.000; //// -10.645267

  //// select IMUs until last one timestamp > DVL timestamp
  std::vector<std::tuple<double,Eigen::Vector3d,Eigen::Vector3d>> imu_init;
  
  double last_dvl_time = std::get<0>(dvl_data.back()) + t_offset;
  //// copy cooresponding IMU 
  auto selected_frame = std::find_if(imu_data.begin(), imu_data.end(),
                   [&](const auto& imu){return std::get<0>(imu) > last_dvl_time ;});
  if(selected_frame != imu_data.end())
    std::copy(imu_data.begin(), selected_frame+1, std::back_inserter(imu_init));
  else{
    std::cout<<"No IMU found for DVl time\n";
    return 0;
  }  
  // // TEST:
  // printf("\n\n");
  // for(const auto &imu:imu_init)
  //   std::cout<<std::setprecision(17)<<"imu: "<<std::get<0>(imu)<<std::endl;

  //// calculate acceleration and do linear interpolate with IMU timestamp: time, accel, velocity
  std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> dvl_init;
  linearInterp(t_offset, imu_init, dvl_data, dvl_init);
  // // TEST
  // for(const auto dvl:dvl_init)
  //   std::cout<<std::setprecision(17)<<"t: "<<std::get<0>(dvl)<<std::setprecision(7)<<
  //     " a: "<< std::get<1>(dvl).x()<<" "<< std::get<1>(dvl).y()<<" "<< std::get<1>(dvl).z()<<
  //     " v: "<< std::get<2>(dvl).x()<<" "<< std::get<2>(dvl).y()<<" "<< std::get<2>(dvl).z()<<"\n";


  Eigen::Matrix3d R_I_D;
  Eigen::Vector3d p_I_D;
  R_I_D << -0.99993,   0.0113624,-0.0032232,
          0.0113304, 0.999888,  0.00975245,
           0.0113304, 0.999888,  0.00975245;
  p_I_D <<  0.433828,0.0861796,-0.302024;

  Eigen::Vector3d ba_avg;
  //// first data is not using since the vehicle is assuming not moving
  for(int i=1; i<dvl_init.size(); i++) {
    /*** Estimate DVL measured acceleration in IMU frame: 
     *** Troni and Whitcomb JFR 2015, Eq.12
    ***  a_I_hat = [w_a]x R_I_D v_D + R_I_D a_D - ([w_a]x^2 + [w_a_dot]x) p_I_D
    ***/
    auto skew = msckf_dvio::toSkewSymmetric(std::get<2>(imu_init.at(i)));
    auto skew_sq = skew * skew;
    //// calculate derivative of angular velocity: w_a_dot, then do skew symetric
    auto skew_dot = msckf_dvio::toSkewSymmetric( 
                     (std::get<2>(imu_init.at(i)) - std::get<2>(imu_init.at(i-1))) /
                     (std::get<0>(imu_init.at(i)) - std::get<0>(imu_init.at(i-1))) );

    auto a_I_hat = skew * R_I_D *std::get<2>(dvl_init.at(i))  + 
                   R_I_D * std::get<1>(dvl_init.at(i)) -
                   (skew_sq + skew_dot) * p_I_D;
    //// compensate IMU acceleration by DVL measurement
    auto a_compensated = std::get<1>(imu_init.at(i)) - a_I_hat;

    /*** Acceleration Bias Estimation  ***/

    //// Normalize the gravity z-axis that projected into IMU frame:
    ////   cosine of inertial frame z-axis (gravity align with z-axis) with IMU frame's x-axis,y-axis and x-axis
    Eigen::Vector3d z_I_G = a_compensated / a_compensated.norm();
    
    //// Normalize the gravity x-axis that projected into IMU frame:
    ////    Get x-axis to perpendicular to z-axis
    ////    Use [Gram-Schmidt Process](https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
    Eigen::Vector3d x_I(1, 0, 0);
    Eigen::Vector3d x_I_G = x_I - z_I_G * z_I_G.transpose() * x_I;
    x_I_G = x_I_G / x_I_G.norm();

    //// Normalize the gravity y-axis that projected into IMU frame:
    ////    Get y from the cross product of these two
    Eigen::Vector3d y_I_G = msckf_dvio::toSkewSymmetric(z_I_G) * x_I_G;

    // From these axes get rotation
    Eigen::Matrix3d R_I_G;
    R_I_G.block(0, 0, 3, 1) = x_I_G;
    R_I_G.block(0, 1, 3, 1) = y_I_G;
    R_I_G.block(0, 2, 3, 1) = z_I_G;

    Eigen::Vector3d ba = a_compensated - R_I_G * Eigen::Vector3d(0.0,0.0,9.81);

    ba_avg += ba;
  }
  ba_avg = ba_avg/(dvl_init.size()-1);
  // // TEST
  // std::cout<<"ba_avg: "<< ba_avg<<"\n";
    
  /*** Find rotation matrix between local IMU frame and global interial frame ***/
  //// use last IMU data in initialization buffer to estimate rotation matrix
    
  /*** Gyro Bias Estimation  ***/
  //// use previous data before jump to estimate gyro bias
}

// 99 avg : ba_avg    
// 0.0027382
// -0.000650351
// 0.0104784

// manual:
// b_g:
//   -0.001
//   0.002
// -0.0002

// b_a: 
// -5.0e-05
//  4.0e-05
//  0.005
