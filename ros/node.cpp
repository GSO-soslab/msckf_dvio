#include "node.h"

#include <thread>

namespace msckf_dvio
{

RosNode::RosNode(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  // get parameters and feed to manager system
  loadParamSystem(parameters);
  loadParamInit(parameters);
  loadParamPrior(parameters);
  loadParamImage(parameters);
  parameters.printParam();

  manager = std::make_shared<MsckfManager>(parameters);
  visualizer = std::make_shared<RosVisualizer>(nh, manager);

  // ROS subscribers
  for(const auto& sensor : parameters.sys.sensors) {

    switch(sensor) {
      case Sensor::IMU: {
        ros::Subscriber sub = nh_.subscribe(parameters.sys.topics[sensor], 2000, &RosNode::imuCallback, this);
        auto sub_ptr = std::make_shared<ros::Subscriber>(sub);
        subscribers[sensor] = sub_ptr;
        break;
      }

      case Sensor::DVL: {
        ros::Subscriber sub = nh_.subscribe(parameters.sys.topics[sensor], 100, &RosNode::dvlCallback, this);
        auto sub_ptr = std::make_shared<ros::Subscriber>(sub);
        subscribers[sensor] = sub_ptr;
        break;
      }

      case Sensor::PRESSURE: {
        ros::Subscriber sub = nh_.subscribe(parameters.sys.topics[sensor], 100, &RosNode::pressureCallback, this);
        auto sub_ptr = std::make_shared<ros::Subscriber>(sub);
        subscribers[sensor] = sub_ptr;
        break;
      }

      case Sensor::CAM0: {
        ros::Subscriber sub = nh_.subscribe(parameters.sys.topics[sensor], 200, &RosNode::imageCallback, this);
        auto sub_ptr = std::make_shared<ros::Subscriber>(sub);
        subscribers[sensor] = sub_ptr;
        break;
      }

      case Sensor::DVL_CLOUD: {
        ros::Subscriber sub = nh_.subscribe(parameters.sys.topics[sensor], 100, &RosNode::dvlCloudCallback, this);
        auto sub_ptr = std::make_shared<ros::Subscriber>(sub);
        subscribers[sensor] = sub_ptr;        
        break;
      }

      case Sensor::CAM0_FEATURE: {
        ros::Subscriber sub = nh_.subscribe(parameters.sys.topics[sensor], 100, &RosNode::featureCallback, this);
        auto sub_ptr = std::make_shared<ros::Subscriber>(sub);
        subscribers[sensor] = sub_ptr;        
        break;
      }

      default:
        ROS_WARN("some unknown sensor: %s", enumToString(sensor).c_str());
        break;
    }
  }

  service_ = nh_.advertiseService("cmd",&RosNode::srvCallback, this);

  // load some test data if need
  // loadCSV();
}    

void RosNode::loadParamSystem(Params &params) {
  // ==================== System ==================== //

  // load 
  std::vector<std::string> sensors;
  XmlRpc::XmlRpcValue rosparam_topics;
  XmlRpc::XmlRpcValue rosparam_buffers;

  nh_private_.param<int>("SYS/backend_hz", params.sys.backend_hz, 20);
  nh_private_.getParam("SYS/sensors", sensors);
  nh_private_.getParam("SYS/topics", rosparam_topics);
  nh_private_.getParam("SYS/buffers", rosparam_buffers);
  nh_private_.param<std::string>("SYS/csv", params.sys.csv, "not_set.csv");

  ROS_ASSERT(rosparam_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(rosparam_buffers.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // convert for sensor 
  for(const auto& name : sensors) {
    // convert
    auto sensor_enum = stringToEnum(name, Sensor::NONE);
    // check
    if(sensor_enum == Sensor::NONE) {
      ROS_ERROR("can't parse this sensor name = %s !!!", name.c_str());
    }
    else {
      params.sys.sensors.push_back(sensor_enum);
    }
  }

  // convert for sensor topic
  for(uint32_t i = 0 ; i < rosparam_topics.size() ; i++) {
      for(const auto& name : params.sys.sensors) {
          // convert sensor name enum to string
          auto key = enumToString(name);
          // check if this sensor exist
          if(rosparam_topics[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
              continue;
          }
          // save to parameters
          auto topic = static_cast<std::string>(rosparam_topics[i][key]);
          params.sys.topics[name] = topic;

          // each line only has one sensor, so found and break
          break;
      }
  }

  if(params.sys.topics.size() != params.sys.sensors.size()) {
    ROS_ERROR("not enough rostopics for the given sensors");
  }

  // convert for sensor buffers
  for(uint32_t i = 0 ; i < rosparam_buffers.size() ; i++) {
      for(const auto& name : params.sys.sensors) {
          // convert sensor name enum to string
          auto key = enumToString(name);
          // check if this sensor exist
          if(rosparam_buffers[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
              continue;
          }          
          // save to parameters
          auto buffer = static_cast<int>(rosparam_buffers[i][key]);   
          params.sys.buffers[name] = buffer;

          // each line only has one sensor, so found and break
          break;
      }
  }

  // ==================== MSCKF ==================== //

  // if this sensor exist 
  // DVL:
  // CAM0: 

  nh_private_.param<bool>("MSCKF/dvl_exterisic_R", params.msckf.do_R_I_D,    false);
  nh_private_.param<bool>("MSCKF/dvl_exterisic_p", params.msckf.do_p_I_D,    false);
  nh_private_.param<bool>("MSCKF/dvl_timeoffset",  params.msckf.do_time_I_D, false);
  nh_private_.param<bool>("MSCKF/dvl_scale",       params.msckf.do_scale_D,  false);
  nh_private_.param<int> ("MSCKF/dvl_clone",       params.msckf.max_clone_D, 0);

  nh_private_.param<bool>("MSCKF/cam_exterisic_R", params.msckf.do_R_C_I,    false);
  nh_private_.param<bool>("MSCKF/cam_exterisic_p", params.msckf.do_p_C_I,    false);
  nh_private_.param<bool>("MSCKF/cam_timeoffset",  params.msckf.do_time_C_I, false);
  nh_private_.param<int> ("MSCKF/cam_clone",       params.msckf.max_clone_C, 0);

  nh_private_.param<int> ("MSCKF/max_msckf_update", params.msckf.max_msckf_update, 40);

  nh_private_.getParam("MSCKF/marginalized_clone", params.msckf.marginalized_clone);

  // check if given index outside of slide window size
  // for(const auto& clone : params.msckf.marginalized_clone) {
  //   if(clone > params.msckf.max_clone_C -1) {
  //     ROS_ERROR("marginalized_clone: given marg clone index out of clone window");
  //   }
  // }

  if(params.msckf.marginalized_clone.size() == 0) {
    // if no indexs given, assumming use the entire slide window
    for(int i =0; i < params.msckf.max_clone_C; i++){
      params.msckf.marginalized_clone.push_back(i);
    }
  }
  else {
    // check if given index outside of slide window size
    for(const auto& clone : params.msckf.marginalized_clone) {
      if(clone > params.msckf.max_clone_C -1) {
        ROS_ERROR("marginalized_clone: given marg clone index out of clone window");
      }
    }
  }
}

void RosNode::loadParamInit(Params &params) {
  // ==================== Initialization ==================== //

  std::string mode_str;
  nh_private_.param<std::string> ("INIT_MODE", mode_str, enumToString(InitMode::INIT_NONE));

  params.init.mode = stringToEnum(mode_str, InitMode::INIT_NONE);


  switch(params.init.mode) {

    case InitMode::INIT_DVL_PRESSURE: {
      // check if the param is set to right enum type
      auto param_name = enumToString(InitMode::INIT_DVL_PRESSURE);
      if (!nh_private_.hasParam(param_name)) {
        ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
      }

      nh_private_.param<int>   (param_name + "/imu_window",        params.init.dvl_pressure.imu_window,        20);
      nh_private_.param<double>(param_name + "/imu_var",           params.init.dvl_pressure.imu_var,           0.2);
      nh_private_.param<double>(param_name + "/imu_delta",         params.init.dvl_pressure.imu_delta,         0.07);
      nh_private_.param<int>   (param_name + "/dvl_window",        params.init.dvl_pressure.dvl_window,        4);
      nh_private_.param<double>(param_name + "/dvl_delta",         params.init.dvl_pressure.dvl_delta,         0.05);
      nh_private_.param<double>(param_name + "/dvl_init_duration", params.init.dvl_pressure.dvl_init_duration, 1.0);
      break;
    }

    case InitMode::INIT_SETTING: {
      // ---------- check init mode ---------- //

      auto param_name = enumToString(InitMode::INIT_SETTING);
      if (!nh_private_.hasParam(param_name)) {
        ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
      }

      // ---------- Load initialized param for each sensors ---------- //

      // check sensor avaiable 
      for(const auto& sensor : params.sys.sensors) {
        auto sensor_param = param_name + "/" + enumToString(sensor);
        if (!nh_private_.hasParam(sensor_param)) {
          continue;
        }

        switch(sensor) {
          case Sensor::IMU: {

            // check the parameter that must exist
            if (nh_private_.hasParam(sensor_param + "/time")) {
              nh_private_.getParam(sensor_param + "/time", params.init.setting[Sensor::IMU].time);
            }
            else {
              ROS_ERROR("%s: does not has timestamp", enumToString(sensor).c_str());
            }

            // check the parameters may exist            
            if (nh_private_.hasParam(sensor_param + "/state")) {
              nh_private_.getParam(sensor_param + "/state", params.init.setting[Sensor::IMU].state);
              if(params.init.setting[Sensor::IMU].state.size() != 16) {
                ROS_ERROR("parameter init: bad size for IMU state");
              }
            }

            if (nh_private_.hasParam(sensor_param + "/temporal")) {
              nh_private_.getParam(sensor_param + "/temporal", params.init.setting[Sensor::IMU].temporal);
            }

            if (nh_private_.hasParam(sensor_param + "/extrinsic")) {
              nh_private_.getParam(sensor_param + "/extrinsic", params.init.setting[Sensor::IMU].extrinsic);
            }

            if (nh_private_.hasParam(sensor_param + "/intrinsic")) {
              nh_private_.getParam(sensor_param + "/intrinsic", params.init.setting[Sensor::IMU].intrinsic);
            }

            if (nh_private_.hasParam(sensor_param + "/global")) {
              nh_private_.getParam(sensor_param + "/global", params.init.setting[Sensor::IMU].global);
            }

            break;
          }

          case Sensor::DVL: {

            // check the parameter that must exist
            if (nh_private_.hasParam(sensor_param + "/time")) {
              nh_private_.getParam(sensor_param + "/time", params.init.setting[Sensor::DVL].time);
            }
            else {
              ROS_ERROR("%s: does not has timestamp", enumToString(sensor).c_str());
            }

            // check the parameters may exist            
            if (nh_private_.hasParam(sensor_param + "/state")) {
              nh_private_.getParam(sensor_param + "/state", params.init.setting[Sensor::DVL].state);
            }

            if (nh_private_.hasParam(sensor_param + "/temporal")) {
              nh_private_.getParam(sensor_param + "/temporal", params.init.setting[Sensor::DVL].temporal);
            }

            if (nh_private_.hasParam(sensor_param + "/extrinsic")) {
              nh_private_.getParam(sensor_param + "/extrinsic", params.init.setting[Sensor::DVL].extrinsic);
            }

            if (nh_private_.hasParam(sensor_param + "/intrinsic")) {
              nh_private_.getParam(sensor_param + "/intrinsic", params.init.setting[Sensor::DVL].intrinsic);
            }

            if (nh_private_.hasParam(sensor_param + "/global")) {
              nh_private_.getParam(sensor_param + "/global", params.init.setting[Sensor::DVL].global);
            }

            break;
          }

          case Sensor::PRESSURE: {
            // check the parameter that must exist
            if (nh_private_.hasParam(sensor_param + "/time")) {
              nh_private_.getParam(sensor_param + "/time", params.init.setting[Sensor::PRESSURE].time);
            }
            else {
              ROS_ERROR("%s: does not has timestamp", enumToString(sensor).c_str());
            }

            // check the parameters may exist            
            if (nh_private_.hasParam(sensor_param + "/state")) {
              nh_private_.getParam(sensor_param + "/state", params.init.setting[Sensor::PRESSURE].state);
            }

            if (nh_private_.hasParam(sensor_param + "/temporal")) {
              nh_private_.getParam(sensor_param + "/temporal", params.init.setting[Sensor::PRESSURE].temporal);
            }

            if (nh_private_.hasParam(sensor_param + "/extrinsic")) {
              nh_private_.getParam(sensor_param + "/extrinsic", params.init.setting[Sensor::PRESSURE].extrinsic);
            }

            if (nh_private_.hasParam(sensor_param + "/intrinsic")) {
              nh_private_.getParam(sensor_param + "/intrinsic", params.init.setting[Sensor::PRESSURE].intrinsic);
            }

            if (nh_private_.hasParam(sensor_param + "/global")) {
              nh_private_.getParam(sensor_param + "/global", params.init.setting[Sensor::PRESSURE].global);
            }

            break;
          }

          case Sensor::CAM0_FEATURE: 
          case Sensor::CAM0: {
            break;
          }

          default:
            break;
        }

      }

      break;
    }  

    case InitMode::INIT_NONE:
    default:
      ROS_ERROR("Init Mode name=%s can't be parsed !!!", mode_str.c_str());
      break;
  }
}

void RosNode::loadParamPrior(Params &params) {

  for(const auto& sensor : params.sys.sensors) {

    // loop each parameters for each sensor we selected
    switch(sensor) {
        case Sensor::IMU: {
          // check if the param is set to right enum type
          auto param_name = enumToString(Sensor::IMU);
          if (!nh_private_.hasParam(param_name)) {
            ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
          }

          double gravity;
          nh_private_.param<double>(param_name + "/gravity",                     gravity,                   9.81);
          nh_private_.param<double>(param_name + "/accelerometer_noise_density", params.prior_imu.sigma_a,  2.0000e-3);
          nh_private_.param<double>(param_name + "/accelerometer_random_walk",   params.prior_imu.sigma_ab, 3.0000e-03);
          nh_private_.param<double>(param_name + "/gyroscope_noise_density",     params.prior_imu.sigma_w,  1.6968e-04);
          nh_private_.param<double>(param_name + "/gyroscope_random_walk",       params.prior_imu.sigma_wb, 1.9393e-05);
          params.prior_imu.gravity << 0, 0, gravity;      
          
          break;
        }
        
        case Sensor::DVL: {
          // check if the param is set to right enum type
          auto param_name = enumToString(Sensor::DVL);
          if (!nh_private_.hasParam(param_name)) {
            ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
          }

          //// load parameters from yaml
          XmlRpc::XmlRpcValue rosparam_dvl;
          std::vector<double> noise_bt(3);
          nh_private_.getParam     (param_name + "/T_I_D",            rosparam_dvl);
          nh_private_.param<double>(param_name + "/timeoffset_I_D",   params.prior_dvl.timeoffset, 0.0);
          nh_private_.param<double>(param_name + "/scale",            params.prior_dvl.scale, 1.0);
          nh_private_.getParam     (param_name + "/noise_bt",         noise_bt);

          //// convert to parameters
          Eigen::Matrix4d T_I_D;
          ROS_ASSERT(rosparam_dvl.getType() == XmlRpc::XmlRpcValue::TypeArray);
          for (int32_t i = 0; i < rosparam_dvl.size(); ++i) {
            for(int32_t j=0; j<rosparam_dvl[i].size(); ++j) 
              T_I_D(i,j) = static_cast<double>(rosparam_dvl[i][j]);
          }

          params.prior_dvl.extrinsics.block(0, 0, 4, 1) = toQuaternion(T_I_D.block(0, 0, 3, 3));
          params.prior_dvl.extrinsics.block(4, 0, 3, 1) = T_I_D.block(0, 3, 3, 1);
          params.prior_dvl.sigma_bt << noise_bt.at(0), noise_bt.at(1), noise_bt.at(2);

          break;
        }

        case Sensor::PRESSURE: {
          // check if the param is set to right enum type
          auto param_name = enumToString(Sensor::PRESSURE);
          if (!nh_private_.hasParam(param_name)) {
            ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
          }

          // load 
          nh_private_.param<double>(param_name + "/mount_angle",      params.prior_pressure.mount_angle, 0.0);
          nh_private_.param<double>(param_name + "/noise_pressure",   params.prior_pressure.sigma_pressure, 0.0);
          break;
        }

        case Sensor::CAM0: 
        case Sensor::CAM0_FEATURE: 
        {
          // check if the param is set to right enum type
          auto param_name = enumToString(Sensor::CAM0);
          if (!nh_private_.hasParam(param_name)) {
            ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
          }

          // load
          XmlRpc::XmlRpcValue rosparam_cam;
          std::vector<double> distortion_coeffs(4);
          std::vector<double> intrinsics(4);
          std::vector<double> resolution(2);
          nh_private_.getParam     (param_name + "/T_C_I",             rosparam_cam);
          nh_private_.getParam     (param_name + "/distortion_coeffs", distortion_coeffs);
          nh_private_.getParam     (param_name + "/intrinsics",        intrinsics);
          nh_private_.getParam     (param_name + "/resolution",        resolution);
          nh_private_.param<double>(param_name + "/timeoffset_C_I",    params.prior_cam.timeoffset, 0.0);
          nh_private_.param<double>(param_name + "/noise",    params.prior_cam.noise, 1.0);

          //// convert matrix into pose 
          Eigen::Matrix4d T_C_I;
          ROS_ASSERT(rosparam_cam.getType() == XmlRpc::XmlRpcValue::TypeArray);
          for (int32_t i = 0; i < rosparam_cam.size(); ++i) {
            for(int32_t j=0; j<rosparam_cam[i].size(); ++j) 
              T_C_I(i,j) = static_cast<double>(rosparam_cam[i][j]);
          }

          params.prior_cam.extrinsics.block(0, 0, 4, 1) = toQuaternion(T_C_I.block(0, 0, 3, 3));
          params.prior_cam.extrinsics.block(4, 0, 3, 1) = T_C_I.block(0, 3, 3, 1);
          params.prior_cam.distortion_coeffs << distortion_coeffs.at(0), distortion_coeffs.at(1), 
                                                distortion_coeffs.at(2), distortion_coeffs.at(3);
          params.prior_cam.intrinsics << intrinsics.at(0), intrinsics.at(1), 
                                        intrinsics.at(2), intrinsics.at(3);
          params.prior_cam.image_wh = std::make_pair(resolution.at(0), resolution.at(1));
          
          break;
        }

        default:
          ROS_WARN("Sensor:%s doesn't has prior parameters", enumToString(sensor).c_str());
          break;
      }
  }
}

void RosNode::loadParamImage(Params &params) {

  // ==================== Tracking ==================== //
  std::string mode_str;
  nh_private_.param<std::string> ("TRACK/mode", mode_str, enumToString(TrackMode::TRACK_NONE));
  params.tracking.basic.mode = stringToEnum(mode_str, TrackMode::TRACK_NONE);

  // load basic
  nh_private_.param<int>   ("TRACK/num_aruco",        params.tracking.basic.num_aruco,        1024);
  nh_private_.param<bool>  ("TRACK/downsize_aruco",   params.tracking.basic.downsize_aruco,   false);
  nh_private_.param<bool>  ("TRACK/use_stereo",       params.tracking.basic.use_stereo,       false);
  nh_private_.param<int>   ("TRACK/max_camera",       params.tracking.basic.max_camera,       1);
  nh_private_.param<int>   ("TRACK/cam_id",           params.tracking.basic.cam_id,           0);
  nh_private_.param<double>("TRACK/downsample_ratio", params.tracking.basic.downsample_ratio, 1.0);

  // load specific tracking parameters
  switch(params.tracking.basic.mode) {

    case TrackMode::TRACK_FEATURE: {
      // check if the param is set to right enum type
      auto param_name = enumToString(TrackMode::TRACK_FEATURE);
      if (!nh_private_.hasParam(param_name)) {
        ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
      }      

      // load
      nh_private_.param<int>(param_name + "/todo", params.tracking.feature.todo, 1);
      break;
    }

    case TrackMode::TRACK_KLT: {
      // check if the param is set to right enum type
      auto param_name = enumToString(TrackMode::TRACK_KLT);
      if (!nh_private_.hasParam(param_name)) {
        ROS_ERROR("The yaml setup is not right, should be = %s", param_name.c_str());
      }
      
      // load
      nh_private_.param<int>(param_name + "/num_pts",        params.tracking.klt.num_pts,        250);
      nh_private_.param<int>(param_name + "/fast_threshold", params.tracking.klt.fast_threshold, 15);
      nh_private_.param<int>(param_name + "/grid_x",         params.tracking.klt.grid_x,         5);
      nh_private_.param<int>(param_name + "/grid_y",         params.tracking.klt.grid_y,         3);
      nh_private_.param<int>(param_name + "/min_px_dist",    params.tracking.klt.min_px_dist,    8);
      nh_private_.param<int>(param_name + "/pyram",          params.tracking.klt.pyram,          3);
      break;
    }

    case TrackMode::TRACK_DESCRIPTOR: {
      break;
    }

    case TrackMode::TRACK_NONE:
    default:
      ROS_ERROR("Track Mode name=%s can't be parsed !!!", mode_str.c_str());
      break;    
  }
  
  // ==================== Triangualtion ==================== //

  nh_private_.param<double>("Feature/max_cond_number",  params.triangualtion.max_cond_number, 10000);
  nh_private_.param<double>("Feature/min_dist",         params.triangualtion.min_dist,        0.10);
  nh_private_.param<double>("Feature/max_dist",         params.triangualtion.max_dist,        60);
  nh_private_.param<double>("Feature/lam_mult",         params.triangualtion.lam_mult,        10);
  nh_private_.param<double>("Feature/init_lamda",       params.triangualtion.init_lamda,      0.001);
  nh_private_.param<int>   ("Feature/max_runs",         params.triangualtion.max_runs,        5);
  nh_private_.param<double>("Feature/max_lamda",        params.triangualtion.max_lamda,       1e10);
  nh_private_.param<double>("Feature/min_dx",           params.triangualtion.min_dx,          1e-6);
  nh_private_.param<double>("Feature/min_dcost",        params.triangualtion.min_dcost,       1e-6);
  nh_private_.param<double>("Feature/max_baseline",     params.triangualtion.max_baseline,    40);

  // ==================== Keyframe ==================== //

  nh_private_.param<int>   ("Keyframe/frame_count",     params.keyframe.frame_count,     5);
  nh_private_.param<double>("Keyframe/frame_motion",    params.keyframe.frame_motion,    0.1);
  nh_private_.param<int>   ("Keyframe/motion_space",    params.keyframe.motion_space,    3);
  nh_private_.param<int>   ("Keyframe/min_tracked",     params.keyframe.min_tracked,     50);
  nh_private_.param<double>("Keyframe/scene_ratio",     params.keyframe.scene_ratio,     0.8);  
  nh_private_.param<double>("Keyframe/adaptive_factor", params.keyframe.adaptive_factor, 0.333);  
  nh_private_.param<int>   ("Keyframe/adaptive_power",  params.keyframe.adaptive_power,  1);  
}

void RosNode::loadCSV() {
  // check if we are actulally loading this test file  
  std::filesystem::path filepath = parameters.sys.csv;
  bool exist = std::filesystem::is_directory(filepath.parent_path());
  if(!exist) {
    printf("csv path not exist, no test file given\n");
    return ;
  }

  // load the csv file
  rapidcsv::Document doc(parameters.sys.csv);
  std::vector<size_t> col_id = doc.GetColumn<size_t>("ID");
  std::vector<float> col_fx = doc.GetColumn<float>("p_fx");
  std::vector<float> col_fy = doc.GetColumn<float>("p_fy");
  std::vector<float> col_fz = doc.GetColumn<float>("p_fz");
  assert(col_id.size() == col_fx.size());
  assert(col_fx.size() == col_fy.size());
  assert(col_fy.size() == col_fz.size());
  printf("\n Load test data size: %ld\n", col_id.size());

  // save
  std::unordered_map<size_t, Eigen::Vector3d> truth_feature;
  for(size_t i = 0; i < col_id.size(); i++ ) {
    truth_feature.insert({col_id.at(i), 
                          Eigen::Vector3d(col_fx.at(i), col_fy.at(i), col_fz.at(i))});
  } 

  // pass to the system manager
  manager->setupTest(truth_feature);
}

bool RosNode::srvCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {
  res.success = true;
  res.message = "received";

  return true;
}

void RosNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  //! NOTE: IMU timestamp is not stable: 100hz, actually is duraction is about 0.11,0.11,0.6,0.11,0.11,0.6

  ImuMsg message;
  
  message.time = msg->header.stamp.toSec();
  message.a << msg->linear_acceleration.x, 
               msg->linear_acceleration.y, 
               msg->linear_acceleration.z;
  message.w << msg->angular_velocity.x, 
               msg->angular_velocity.y, 
               msg->angular_velocity.z;

  manager->feedImu(message);
}

void RosNode::dvlCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg) {
  //! TODO: simple fliter here, remove bad velocity measurement 
  //! TODO: we should think a better way to this diagnosis, like using goodbeams, FOM??

  // filter bad measurement when DVL is not using right
  if(msg->twist.twist.linear.x == -32.768f ||
     msg->twist.twist.linear.y == -32.768f ||
     msg->twist.twist.linear.z == -32.768f ) {

    ROS_WARN("ROS node warning: DVL velocity:x=%f,y=%f,z=%f, drop it!\n", 
              msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
    return;
  }

  DvlMsg message;
  message.time = msg->header.stamp.toSec();
  message.v << msg->twist.twist.linear.x, 
               msg->twist.twist.linear.y, 
               msg->twist.twist.linear.z;

  // simple fliter to remove bad time data
  if(message.time > last_t_dvl){
    manager->feedDvl(message);
    last_t_dvl = message.time;
  }
  else{
    ROS_WARN("Node: bad DVL time, drop it!");
  }

}

void RosNode::featureCallback(const sensor_msgs::PointCloud::ConstPtr &msg) {

  FeatureMsg feature_msg;

  // get time
  feature_msg.time = msg->header.stamp.toSec();
  // get u,v,id
  for (size_t i = 0; i < msg->points.size(); i++) {
    feature_msg.u.push_back(static_cast<float>(msg->points[i].x));
    feature_msg.v.push_back(static_cast<float>(msg->points[i].y));
    feature_msg.id.push_back(static_cast<unsigned int>(msg->channels[0].values[i]));
  }

  manager->feedFeature(feature_msg);
}

// TODO: check if feature tracking in image callback will effect IMU callback(overflow, bad imu-image align)
void RosNode::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {

  cv_bridge::CvImagePtr cv_ptr;
  try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//BGR8,MONO8
  }
  catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  // downsampling
  //! TODO: cv::resize vs. cv::pyrDown
  cv::Mat img;
  int width = cv_ptr->image.cols * parameters.tracking.basic.downsample_ratio;
  int height = cv_ptr->image.rows * parameters.tracking.basic.downsample_ratio;
  cv::resize(cv_ptr->image, img, cv::Size(width, height));

  // feed img
  ImageMsg message;
  message.image = img;
  message.time = msg->header.stamp.toSec();

  // simple fliter to remove bad time image
  if(message.time > last_t_img){
    manager->feedCamera(message);
    last_t_img = message.time;
  }
  else{
    ROS_WARN("Node: bad image time, drop it!");
  }
}

void RosNode::pressureCallback(const sensor_msgs::FluidPressure::ConstPtr &msg) {
  PressureMsg message;
  message.time = msg->header.stamp.toSec();
  message.p = msg->fluid_pressure;

  // simple fliter to remove bad time data
  if(message.time > last_t_pressure){
    manager->feedPressure(message);
    last_t_pressure = message.time;
  }
  else{
    ROS_WARN("Node: bad Pressure time, drop it!");
  }

}

void RosNode::dvlCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  //! TODO: multi-path will has degraded measurement, filter multi-path based on sub_map, plane-constrain
  pcl::PCLPointCloud2 pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_conversions::toPCL(*msg, pc2);
  pcl::fromPCLPointCloud2(pc2, *cloud);

  manager->feedDvlCloud(cloud);

  // printf("DVL pointcloud received at time:%ld, double=%f\n", cloud->header.stamp, (double)(cloud->header.stamp/1000000.0));
  
}

void RosNode::process() {
  // sleep for short time to wait the system initialization
  // std::chrono::milliseconds dura(1000);
  // std::this_thread::sleep_for(dura);

  int sleep_t = 1.0 / parameters.sys.backend_hz * 1000.0;

  while(1) {
    // do the ekf stuff
    auto t1 = std::chrono::high_resolution_clock::now();  

    manager->backend();

    auto t2 = std::chrono::high_resolution_clock::now();  

    visualizer->visualize();

    auto t3 = std::chrono::high_resolution_clock::now();  

    // printf("[Time Cost]: msckf=%.6fs, vis=%.6fs\n", 
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1e-6,
    //     std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() * 1e-6);

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

} // namespace name


int main(int argc, char **argv) {
  ros::init(argc, argv, "MSCKF_DVIO_Node"); 

  ROS_INFO("\n============================================\n"
           "   !!!!!!!!!! MSCKF Node start: !!!!!!!!!! "
           "\n============================================\n");
  
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  msckf_dvio::RosNode node(nh, nh_private);

  std::thread backendThread{&msckf_dvio::RosNode::process, &node};

  ros::spin();

  backendThread.join();

  return 0;
}