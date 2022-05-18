#ifndef MSCKF_CORE_TYPE_PARAMETERS_H_
#define MSCKF_CORE_TYPE_PARAMETERS_H_

namespace msckf_dvio
{

struct noiseImu {
  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab;
  
  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb;
};

struct paramMsckf {
  // enable DVL exterisic rotation calibration 
  bool do_R_I_D;
  // enable DVL exterisic translation calibration 
  bool do_p_I_D;
  // enable DVL time offset calibration
  bool do_time_I_D;
  // enable DVL scale calibration
  bool do_scale_D;
  // max clone for DVL 
  int max_clone_D;
};

struct Params{

/***** Estimation *****/

/***** Tracker *****/

/***** Feature Triangulation *****/

/***********************/
/***** Calibration ****/
/**********************/

  //==================== IMU ====================/
  // gravity field
  double gravity;
  // Accelerometer noise density (continuous-time) 
  double accl_noise;
  // Accelerometer bias random walk  
  double accl_random_walk;
  // Gyroscope noise density
  double gyro_noise;
  // Gyroscope bias random walk 
  double gyro_random_walk;

  //==================== DVL ======================// 
  //// DVL extrinsic transformation between IMU and DVl (q_I_D, p_I_D).
  Eigen::VectorXd dvl_extrinsics;
  // timeoffset between IMU and DVL
  double timeoffset_I_D;
  // Scale factor for DVL: effected by sound speed
  double scale; 

  //==================== Camera ====================//
  // extrinsic transformation between IMU and Camera
    // Pose pose_I_C; // check the derivative**
  // timeoffset between IMU and Camera
  double timeoffset_I_C;

/******************/
/***** System *****/
/******************/

  // Frequency for backend processing(MSCKF update), increase if more data need for processing
  int backend_hz;
  // Estimate IMU bias and find out the rotation matrix between inertial world frame and IMU body frame using different method
  int imu_init_mode;
  // how many IMU data is selected to detect IMU jump (suddenly move)
  int imu_windows;
  // the IMU variance threshold that indicates IMU jump inclued
  double imu_var;
  // IMU accleration difference in x-axis(forward direction) shows at the suddenly movement time
  double imu_delta;
  // how many DVL data is selected to detect DVL jump (suddenly move)
  int dvl_windows;
  // DVL velocity difference in x-axis(forward direction) shows at the suddenly movement time
  double dvl_delta;

/***** MSCKF State setting *****/
  paramMsckf msckf;
};




} // namespace msckf_dvio

#endif // MSCKF_CORE_TYPE_PARAMETERS_H_