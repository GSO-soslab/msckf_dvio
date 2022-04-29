


struct Params{

/***** Estimation *****/

/***** Tracker *****/

/***** Feature Triangulation *****/

/***** Calibration *****/

  //===== IMU =====/
  // gravity field
  double gravity;

  //===== DVL =======// 
  //// DVL extrinsic transformation between IMU and DVl (q_I_D, p_I_D).
  Eigen::VectorXd dvl_extrinsics;
  // Pose pose_I_D; ?

  // timeoffset between IMU and DVL
  double timeoffset_I_D;
  // Scale factor for DVL: effected by sound speed
  double scale; 

  //===== Camera =====//
  // extrinsic transformation between IMU and Camera
    // Pose pose_I_C; // check the derivative**
  // timeoffset between IMU and Camera
  double timeoffset_I_C;

/***** Noise *****/

/***** System *****/
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
};

// struct noiseParams {
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     double u_var_prime, v_var_prime;
//     Eigen::Matrix<double, 12, 12> Q_imu;
//     Eigen::Matrix<double, 15, 15> initial_imu_covar;
// };

// struct msckfParams {
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     double max_gn_cost_norm, min_rcond, translation_threshold;
//     double redundancy_angle_thresh, redundancy_distance_thresh;
//     int min_track_length, max_track_length, max_cam_states;
// };
