#ifndef MSCKF_TYPE_PARAMETERS_H_
#define MSCKF_TYPE_PARAMETERS_H_

#include <iostream>
#include <iomanip>
#include "utils/utils.h"

namespace msckf_dvio
{
//! Single Estimation State name for each Sub State
// 
#define EST_QUATERNION "Quaternion"
#define EST_POSITION "Position"
#define EST_VELOCITY "Velocity"
#define EST_BIAS_G "BiasGyro"
#define EST_BIAS_A "BiasAcce"
#define EST_TIMEOFFSET "Timeoffset"
#define EST_SCALE "Scale"
// the clone state name will be clone time:
// EST_CLONE = "1614971363.850172520"

//! Use the actual sensor state name
enum Sensor{
  NONE = 0,
  IMU,
  DVL,
  PRESSURE,
  CAM0,
  CLONE_DVL,
  CLONE_CAM0,
  DVL_CLOUD,
  CAM0_FEATURE
};

enum InitMode {
  INIT_NONE = 0,
  INIT_SETTING,
  INIT_STATIC,
  INIT_DVL_PRESSURE,
  INIT_CAMERA,
  INIT_DVL_CAMERA
};

enum TrackMode {
  TRACK_NONE = 0,
  TRACK_FEATURE,
  TRACK_KLT,
  TRACK_DESCRIPTOR
};

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

  //! TODO: for initial_covariance
  // Eigen::VectorXd sigma_init;
};

struct priorPressure {
  // the angle that the actual mounting position rotate to the standing position, 
  // used to transfer pressure measurement into DVL frame's Z
  double mount_angle;
  // noise for pressure (standard deviation)
  double sigma_pressure;
};

struct priorCam {
  // extrinsic transformation between Camera and IMU
  Eigen::Matrix<double, 7, 1> extrinsics;
  // intrinsics projection transformation 
  std::vector<double> intrinsics;
  // Eigen::Matrix<double, 4, 1> intrinsics;
  // distortion coeffs for camera
  std::vector<double> distortion_coeffs;
  // Eigen::Matrix<double, 4, 1> distortion_coeffs;
  // width and height of image
  std::pair<int, int> image_wh;
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

  // enable camera exterisic rotation calibration 
  bool do_R_C_I;
  // enable camera exterisic translation calibration 
  bool do_p_C_I;
  // enable camera time offset calibration
  bool do_time_C_I;
  // max clone for camera 
  int max_clone_C;
  // index of marginalzied measurements
  std::vector<int> marg_meas_index;
  // index of marginalzied clones
  std::vector<int> marg_pose_index;

  // the max features used for MSCKF update
  int max_msckf_update;
};


struct paramInitSensor {
  double time;
  std::vector<double> temporal;
  std::vector<double> state;
  std::vector<double> extrinsic;
  std::vector<double> intrinsic;
  std::vector<double> global;
};

struct paramInitStatic {
  double todo;
};

struct paramInitDvlPressure {
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
};

struct paramInitCamera {
  double todo;
};

struct paramInitDvlCamera {
  double todo;
};

struct paramInit {
  InitMode mode;
  std::map<Sensor, paramInitSensor> setting;
  paramInitStatic stationary;
  paramInitDvlPressure dvl_pressure;
  paramInitCamera camera;
  paramInitDvlCamera dvl_camera;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct paramTrackFeature {
  int todo;
};

struct paramTrackKlt {
  int num_pts;
  int fast_threshold;
  int grid_x;
  int grid_y;
  int min_px_dist;
  int pyram;
};

struct paramTrackDescriptor {
  int todo;
};

struct paramTrackBasic {
  TrackMode mode;
  int num_aruco;
  bool downsize_aruco;
  bool use_stereo;
  int max_camera;
  int cam_id;
  double downsample_ratio;  
  int img_enhancement;
};

struct paramTrack {
  paramTrackBasic basic;
  paramTrackFeature feature;
  paramTrackKlt klt;
  paramTrackDescriptor descriptor;
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
  /// Init lambda for Levenberg-Marquardt optimization
  double init_lamda;  
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

  /// adaptive_factor
  double adaptive_factor;
  ///adaptive_power
  int adaptive_power;
};

struct paramEnhancement {
  // select how many DVL pointcloud frames
  int matched_num;
  // standard deviation of z of 4 points of each DVL range measurements
  float standard_deviation; 
};

struct paramSystem {
  // Frequency for backend processing(MSCKF update), increase if more data need for processing
  int backend_hz;
  // sensors
  std::vector<Sensor> sensors;
  // topics
  std::map<Sensor, std::string> topics;
  // test file path
  std::string csv;
  // buffer time duration for each sensors
  std::map<Sensor, double> buffers;
};

//! TODO: set sub-parameters as shared_ptr? 
//!       so the paramters will updated automaticly, used for localization failed case?
struct Params{

/***** Image related *****/

  paramTrack tracking;

  paramTriangulation triangualtion;

  paramKeyframe keyframe;

  paramEnhancement enhancement;
  
/***** Prior ****/

  priorImu prior_imu;

  priorDvl prior_dvl;

  priorPressure prior_pressure;

  priorCam prior_cam;

/******************/
/***** System *****/
/******************/
  paramSystem sys;

  paramInit init;

  paramMsckf msckf;

  void printParam();

};

inline void Params::printParam() {
  /****************************************************************************/  
  /*********************************  System  *********************************/
  /****************************************************************************/  

  std::cout<<"\n================== System Parameters =======================\n";
  std::cout<<"  backend_hz= " << sys.backend_hz << "\n";
  std::cout<<"  csv= " << sys.csv << "\n";

  std::cout<<"  sensors: ";
  for(const auto& sensor : sys.sensors) {
    std::cout<< enumToString(sensor) <<", ";
  }
  std::cout<<"\n";

  std::cout<<  "  topics: \n";
  for(const auto& [name, topic] : sys.topics) {
    std::cout<<"    " << enumToString(name) <<" = " << topic << "\n";
  }

  std::cout<<  "  buffers: \n";
  for(const auto& [name, buffer] : sys.buffers) {
    std::cout<<"    " << enumToString(name) <<" = " << buffer << " seconds\n";
  }  

  std::cout<<"\n================== MSCKF Parameters =======================\n";
  std::cout<<"  do_R_I_D= " << (msckf.do_R_I_D ? "True" : "False" ) << "\n";
  std::cout<<"  do_p_I_D= " << (msckf.do_p_I_D ? "True" : "False" ) << "\n";
  std::cout<<"  do_time_I_D= " << (msckf.do_time_I_D ? "True" : "False" ) << "\n";
  std::cout<<"  do_scale_D= " << (msckf.do_scale_D ? "True" : "False" ) << "\n\n";

  std::cout<<"  do_R_C_I= " << (msckf.do_R_C_I ? "True" : "False" ) << "\n";
  std::cout<<"  do_p_C_I= " << (msckf.do_p_C_I ? "True" : "False" ) << "\n";
  std::cout<<"  do_time_C_I= " << (msckf.do_time_C_I ? "True" : "False" ) << "\n";
  std::cout<<"  max_clone_C= " << msckf.max_clone_C << "\n\n";

  std::cout<<"  max_msckf_update= " << msckf.max_msckf_update << "\n";

  std::cout<<"  marg_meas_index= ";
  for(const auto& clone : msckf.marg_meas_index) {
    std::cout<< clone <<", "; 
  }
  std::cout<<"\n";

  std::cout<<"  marg_pose_index= ";
  for(const auto& clone : msckf.marg_pose_index) {
    std::cout<< clone <<", "; 
  }
  std::cout<<"\n";

  /****************************************************************************/  
  /*********************************  Prior   *********************************/
  /****************************************************************************/ 
  std::cout<<"\n================== Prior Parameters =======================\n";

  for(const auto& sensor : sys.sensors) {
    switch(sensor) {
      case Sensor::IMU: {
        std::cout<< "\n" << enumToString(Sensor::IMU) << "\n";
        std::cout<<"  gravity: "<< prior_imu.gravity.transpose() << "\n";
        std::cout<<"  acce_noise_density: " << prior_imu.sigma_a << "\n";
        std::cout<<"  acce_random_walk: " << prior_imu.sigma_ab << "\n";
        std::cout<<"  gyro_noise_density: " << prior_imu.sigma_w << "\n";
        std::cout<<"  gyro_random_walk: " << prior_imu.sigma_wb << "\n";  
        break;
      }

      case Sensor::DVL: {
        std::cout<<"\n" << enumToString(Sensor::DVL) << "\n";
        Eigen::Matrix4d T_I_D = Eigen::Matrix4d::Identity();
        T_I_D.block(0, 0, 3, 3) = toRotationMatrix(prior_dvl.extrinsics.block(0, 0, 4, 1));
        T_I_D.block(0, 3, 3, 1) = prior_dvl.extrinsics.block(4, 0, 3, 1);
        std::cout<<"  T_I_D: \n" << T_I_D << "\n"; 
        std::cout<<"  timeoffset_I_D: " << prior_dvl.timeoffset << "\n";
        std::cout<<"  scale: " << prior_dvl.scale << "\n";
        std::cout<<"  noise_bt: " << prior_dvl.sigma_bt.transpose() << "\n";
        break;
      }

      case Sensor::PRESSURE: {
        std::cout<<"\n" << enumToString(Sensor::PRESSURE) << "\n";
        std::cout<<"  mount_angle: " << prior_pressure.mount_angle << "\n"; 
        std::cout<<"  noise_pressure: " << prior_pressure.sigma_pressure << "\n";        
        break;
      }

      case Sensor::CAM0: 
      case Sensor::CAM0_FEATURE: {
        std::cout<<"\n" << enumToString(Sensor::CAM0) << "\n";
        Eigen::Matrix4d T_C_I = Eigen::Matrix4d::Identity();
        T_C_I.block(0, 0, 3, 3) = toRotationMatrix(prior_cam.extrinsics.block(0, 0, 4, 1));
        T_C_I.block(0, 3, 3, 1) = prior_cam.extrinsics.block(4, 0, 3, 1);        
        std::cout << "  T_C_I: \n" <<T_C_I << "\n";
        std::cout << "  distortion_coeffs: ";
        for(const auto& i:prior_cam.distortion_coeffs) {
          std::cout<<i <<", ";
        }
        std::cout<<"\n";

        std::cout << "  intrinsics: ";
        for(const auto& i:prior_cam.intrinsics) {
          std::cout<<i <<", ";
        }
        std::cout<<"\n";

        std::cout << "  resolution(W x H): " << prior_cam.image_wh.first << " x " << prior_cam.image_wh.second << "\n";
        std::cout << "  timeoffset_C_I: " << prior_cam.timeoffset << "\n";
        std::cout << "  noise: " << prior_cam.noise << "\n";        
        break;
      }

      default:
        break;
    }
  }

  /****************************************************************************/  
  /*********************************   Init   *********************************/
  /****************************************************************************/ 

  switch(init.mode) {
    case InitMode::INIT_DVL_PRESSURE: {
      std::cout<<"\n================== Init Parameters =======================\n";
      std::cout<<"  init mode: " << enumToString(InitMode::INIT_DVL_PRESSURE) <<"\n";
      std::cout<<"  imu_window: " << init.dvl_pressure.imu_window << "\n";
      std::cout<<"  imu_var: " << init.dvl_pressure.imu_var << "\n";
      std::cout<<"  imu_delta: " << init.dvl_pressure.imu_delta << "\n";
      std::cout<<"  dvl_window: " <<  init.dvl_pressure.dvl_window << "\n";
      std::cout<<"  dvl_delta: " << init.dvl_pressure.dvl_delta << "\n";
      std::cout<<"  dvl_init_duration: " << init.dvl_pressure.dvl_init_duration << "\n";       
      break;
    }

    case InitMode::INIT_SETTING: {
      std::cout<<"\n================== Init Parameters =======================\n";
      std::cout<<"  init mode: " << enumToString(InitMode::INIT_SETTING) <<"\n";

      // print each sensor initialization result
      for(const auto& [sensor, param] : init.setting) {
        // sensor name
        std::cout<< "  " << enumToString(sensor) << "\n";

        std::cout<< std::fixed <<  std::setprecision(9);
        // time
        std::cout<<"    time: " << init.setting[sensor].time << "\n";
        // temporal
        if(init.setting[sensor].temporal.size() > 0) {
          std::cout<<"    temporal: ";
          for(const auto& value : init.setting[sensor].temporal) {
            std::cout<< value <<", ";
          }
          std::cout<<"\n";
        }

        std::cout<< std::fixed <<  std::setprecision(6);
        // state
        if(init.setting[sensor].state.size() > 0) {
          std::cout<<"    state: ";
          for(const auto& value : init.setting[sensor].state) {
            std::cout<< value <<", ";
          }
          std::cout<<"\n";
        }
        // extrinsic
        if(init.setting[sensor].extrinsic.size() > 0) {
          std::cout<<"    extrinsic: ";
          for(const auto& value : init.setting[sensor].extrinsic) {
            std::cout<< value <<", ";
          }
          std::cout<<"\n";
        }
        // intrinsic
        if(init.setting[sensor].intrinsic.size() > 0) {
          std::cout<<"    intrinsic: ";
          for(const auto& value : init.setting[sensor].intrinsic) {
            std::cout<< value <<", ";
          }
          std::cout<<"\n";
        }
        //
        if(init.setting[sensor].global.size() > 0) {
          std::cout<<"    global: ";
          for(const auto& value : init.setting[sensor].global) {
            std::cout<< value <<", ";
          }
          std::cout<<"\n";
        }
      }

      break;
    }

    default:
      break;
  }

  /****************************************************************************/  
  /*********************************  Image   *********************************/
  /****************************************************************************/ 

  // ---------------------- Tracking ---------------------- //

  // basic
  std::cout<<"\n================== Track basic: =======================\n";
  std::cout<<"  Track mode: " << enumToString(tracking.basic.mode) <<"\n";
  std::cout<<"  num_aruco: " << tracking.basic.num_aruco <<"\n";      
  std::cout<<"  downsize_aruco: " << (tracking.basic.downsize_aruco ? "True" : "False") <<"\n";      
  std::cout<<"  use_stereo: " << (tracking.basic.use_stereo ? "True" : "False") <<"\n";      
  std::cout<<"  max_camera: " << tracking.basic.max_camera <<"\n";      
  std::cout<<"  cam_id: " << tracking.basic.cam_id <<"\n";      
  std::cout<<"  downsample_ratio: " << tracking.basic.downsample_ratio <<"\n";      
  std::cout<<"  img_enhancement: " << tracking.basic.img_enhancement <<"\n";      

  switch(tracking.basic.mode) {
    case TrackMode::TRACK_KLT: {
      std::cout<<"\n---------------- KLT ----------------\n";
      std::cout<<"  num_pts: " << tracking.klt.num_pts <<"\n";      
      std::cout<<"  fast_threshold: " << tracking.klt.fast_threshold <<"\n";      
      std::cout<<"  grid_x: " << tracking.klt.grid_x <<"\n";      
      std::cout<<"  grid_y: " << tracking.klt.grid_y <<"\n";      
      std::cout<<"  min_px_dist: " << tracking.klt.min_px_dist <<"\n";      
      std::cout<<"  pyram: " << tracking.klt.pyram <<"\n";      
      break;
    }

    case TrackMode::TRACK_FEATURE: {
      std::cout<<"\n---------------- Feature ----------------\n";
      std::cout<<"  todo: " << tracking.feature.todo <<"\n";      

      break;
    }

    default:
      break;
  }

  // ---------------------- Triangulation ---------------------- //
  std::cout<<"\n================== Triangulation Parameters =======================\n";
  std::cout<<"  max_cond_number: " << triangualtion.max_cond_number <<"\n";   
  std::cout<<"  min_dist: " << triangualtion.min_dist <<"\n";   
  std::cout<<"  max_dist: " << triangualtion.max_dist <<"\n";   
  std::cout<<"  lam_mult: " << triangualtion.lam_mult <<"\n";   
  std::cout<<"  init_lamda: " << triangualtion.init_lamda <<"\n";   
  std::cout<<"  max_runs: " << triangualtion.max_runs <<"\n";   
  std::cout<<"  max_lamda: " << triangualtion.max_lamda <<"\n";   
  std::cout<<"  min_dx: " << triangualtion.min_dx <<"\n";   
  std::cout<<"  min_dcost: " << triangualtion.min_dcost <<"\n";   
  std::cout<<"  max_baseline: " << triangualtion.max_baseline <<"\n";   

  // ---------------------- Keyframe ---------------------- //
  std::cout<<"\n================== Keyframe Parameters =======================\n";
  std::cout<<"  frame_count: " << keyframe.frame_count <<"\n";   
  std::cout<<"  frame_motion: " << keyframe.frame_motion <<"\n";   
  std::cout<<"  motion_space: " << keyframe.motion_space <<"\n";   
  std::cout<<"  min_tracked: " << keyframe.min_tracked <<"\n";   
  std::cout<<"  scene_ratio: " << keyframe.scene_ratio <<"\n";   
  std::cout<<"  adaptive_factor: " << keyframe.adaptive_factor <<"\n";   
  std::cout<<"  adaptive_power: " << keyframe.adaptive_power <<"\n";  

  // ---------------------- Enhancement ---------------------- //
  std::cout<<"\n================== Depth Enhancement =======================\n";
  std::cout<<"  matched_num: " << enhancement.matched_num <<"\n";   
  std::cout<<"  standard_deviation: " << enhancement.standard_deviation <<"\n";   
}



} // namespace msckf_dvio

#endif // MSCKF_TYPE_PARAMETERS_H_