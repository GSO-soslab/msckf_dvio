#ifndef MSCKF_TYPE_PARAMETERS_H_
#define MSCKF_TYPE_PARAMETERS_H_

namespace msckf_dvio
{

struct priorImu {
  /// gravity
  Eigen::Vector3d gravity;
  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a;
  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab;
  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w;
  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb;
};

struct priorDvl {
  // DVL extrinsic transformation between IMU and DVL
  Eigen::Matrix<double, 7, 1> extrinsics;
  // timeoffset between IMU and DVL
  double timeoffset;
  // Scale factor for DVL: effected by sound speed
  double scale; 
  // sound speed: used when not estimate the scale and do sound spped correction
  double sound_speed;
  // BT 3-axis veloicty measurement white noise( standard deviation )
  Eigen::Vector3d sigma_bt;
  // the angle that the actual mounting position rotate to the standing position, 
  // used to transfer pressure measurement into DVL frame's Z
  double mount_angle;
  // noise for pressure (standard deviation)
  double sigma_pressure;

  //! TODO: for initial_covariance
  // Eigen::VectorXd sigma_init;
};

struct priorCam {
  // extrinsic transformation between Camera and IMU
  Eigen::Matrix<double, 7, 1> extrinsics;
  // intrinsics projection transformation 
  Eigen::Matrix<double, 4, 1> intrinsics;
  // distortion coeffs for camera
  Eigen::Matrix<double, 4, 1> distortion_coeffs;
  // timeoffset between Camera and IMU
  double timeoffset;
  // measurement white noise( standard deviation ) in pixel
  double noise;
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

  // enable camera exterisic rotation calibration 
  bool do_R_C_I;
  // enable camera exterisic translation calibration 
  bool do_p_C_I;
  // enable camera time offset calibration
  bool do_time_C_I;
  // max clone for camera 
  int max_clone_C;
  // index of marginalzied clone 
  std::vector<int> marginalized_clone;

  // the max features used for MSCKF update
  int max_msckf_update;
};

enum InitMode {
  SETTING = 0,
  STATIC = 1,
  DVL_PRESSURE = 2,
  CAMERA = 3,
  DVL_CAMERA = 4
};

struct paramInit {
  InitMode mode;
  // Estimate IMU bias and find out the rotation matrix between inertial world frame and IMU body frame using different method
  int imu_init_mode;
  // how many IMU data is selected to detect IMU jump (suddenly move)
  int imu_window;
  // the IMU variance threshold that indicates IMU jump inclued
  double imu_var;
  // IMU accleration difference in x-axis(forward direction) shows at the suddenly movement time
  double imu_delta;
  // how many DVL data is selected to detect DVL jump (suddenly move)
  int dvl_window;
  // DVL velocity difference in x-axis(forward direction) shows at the suddenly movement time
  double dvl_delta;
  // how many second selected for initialization
  double dvl_init_duration;
  // use given init state result or not
  bool init_given;
  // gievn state
  Eigen::Matrix<double, 17, 1> init_state;
};

struct paramTrack {
  int num_aruco;
  int num_pts;
  int fast_threshold;
  int grid_x;
  int grid_y;
  int min_px_dist;
  bool downsize_aruco;
  bool use_stereo;
  int max_camera;
  int pyram;
  int cam_id;
  double downsample_ratio;
};

struct paramTriangulation {
  /// Max condition number of linear triangulation matrix accept triangulated features
  double max_cond_number;
  /// Minimum distance to accept triangulated features
  double min_dist;
  /// Minimum distance to accept triangulated features
  double max_dist;
  /// Multiplier to increase/decrease lambda
  double lam_mult; 
  /// Max runs for Levenberg-Marquardt
  int max_runs; 
  /// Max lambda for Levenberg-Marquardt optimization
  double max_lamda;  
  /// Cutoff for dx increment to consider as converged
  double min_dx; 
  /// Cutoff for cost decrement to consider as converged
  double min_dcost; 
  /// Max baseline ratio to accept triangulated features
  double max_baseline;
};

struct paramKeyframe {
  /// option 1: frame count
  int frame_count;
  /// option 2: relative motion constraint
  double frame_motion;
  /// motion constraint for 3D or 2D
  int motion_space;
  /// option 3: minimum tracked features
  int min_tracked;
  /// option 4: ratio = tracked feature num from last keyframe / total features at current frame
  double scene_ratio;
};

//! TODO: set sub-parameters as shared_ptr? 
//!       so the paramters will updated automaticly, used for localization failed case?
struct Params{

/***** Image frontend *****/

  paramTrack tracking;

  paramTriangulation triangualtion;

  paramKeyframe keyframe;
  
/***** Prior ****/

  priorImu prior_imu;

  priorDvl prior_dvl;

  priorCam prior_cam;

/******************/
/***** System *****/
/******************/

  // Frequency for backend processing(MSCKF update), increase if more data need for processing
  int backend_hz;
  // use this simply filter bad DVL velocity measurement
  double dvl_v_threshold;

  paramInit init;

  paramMsckf msckf;

};




} // namespace msckf_dvio

#endif // MSCKF_TYPE_PARAMETERS_H_