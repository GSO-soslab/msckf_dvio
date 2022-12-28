#include "node.h"

#include <thread>

namespace msckf_dvio
{

RosNode::RosNode(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  // get parameters and feed to manager system
  parameters = loadParameters();

  manager = std::make_shared<MsckfManager>(parameters);
  visualizer = std::make_shared<RosVisualizer>(nh, manager);

  // ROS related
  sub_imu = nh_.subscribe("imu", 2000, &RosNode::imuCallback, this);
  sub_dvl = nh_.subscribe("dvl", 100, &RosNode::dvlCallback, this);
  sub_img = nh_.subscribe("image", 200, &RosNode::imageCallback, this);
  sub_pressure = nh_.subscribe("pressure", 100, &RosNode::pressureCallback, this);
  sub_pointcloud = nh_.subscribe("pointcloud", 100, &RosNode::pointcloudCallback, this);

  service_ = nh_.advertiseService("cmd",&RosNode::srvCallback, this);
}    

Params RosNode::loadParameters() {

  Params params;

  /***************************************************************************************/
  /********************************** System configuration *******************************/
  /***************************************************************************************/

  // ==================== System ==================== //
  std::vector<std::string> sensors;
  nh_private_.param<int>   ("SYS/backend_hz",      params.sys.backend_hz,      20);
  nh_private_.getParam("SYS/sensors", sensors);

  for(const auto& name : sensors) {
    auto sensor = magic_enum::enum_cast<Sensor>(name);
    if(sensor.has_value()) {
      auto sensor_enum = sensor.value();
      params.sys.sensors.push_back(sensor_enum);
    }
    else {
      ROS_ERROR("can't parse this sensor name = %s !!!", name.c_str());
    }
  }

  // ==================== Initialization ==================== //
  int init_mode;

  nh_private_.param<int>   ("INIT_MODE",     init_mode,     2);
  params.init.mode = static_cast<InitMode>(init_mode);

  switch(params.init.mode) {

    case InitMode::DVL_PRESSURE: {
      nh_private_.param<int>   ("INIT_DVL_PRESSURE/imu_window",        params.init.dvl_pressure.imu_window,        20);
      nh_private_.param<double>("INIT_DVL_PRESSURE/imu_var",           params.init.dvl_pressure.imu_var,           0.2);
      nh_private_.param<double>("INIT_DVL_PRESSURE/imu_delta",         params.init.dvl_pressure.imu_delta,         0.07);
      nh_private_.param<int>   ("INIT_DVL_PRESSURE/dvl_window",        params.init.dvl_pressure.dvl_window,        4);
      nh_private_.param<double>("INIT_DVL_PRESSURE/dvl_delta",         params.init.dvl_pressure.dvl_delta,         0.05);
      nh_private_.param<double>("INIT_DVL_PRESSURE/dvl_init_duration", params.init.dvl_pressure.dvl_init_duration, 1.0);
      break;
    }

    case InitMode::SETTING: {
      // --------------- time --------------- //
      XmlRpc::XmlRpcValue rosparam_time;
      nh_private_.getParam("INIT_SETTING/time", rosparam_time);
      ROS_ASSERT(rosparam_time.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for(uint32_t i = 0 ; i < rosparam_time.size() ; i++) {
          for(const auto& name : params.sys.sensors) {
              // convert sensor name enum to string
              auto key = static_cast<std::string>(magic_enum::enum_name(name));

              // check if this sensor time exist
              if(rosparam_time[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                  continue;
              }
              // save to parameters
              auto time = static_cast<double>(rosparam_time[i][key]);
              params.init.setting.time[name] = time;
              // each line only has one sensor, so found and break
              break;
          }
      }

      // --------------- 15 states --------------- //
      // define vector to save parameters
      std::vector<double> orientation(4);
      std::vector<double> position(3);
      std::vector<double> velocity(3);
      std::vector<double> bias_gyro(3);
      std::vector<double> bias_acce(3);
      // load
      nh_private_.getParam("INIT_SETTING/orientation", orientation);
      nh_private_.getParam("INIT_SETTING/position", position);
      nh_private_.getParam("INIT_SETTING/velocity", velocity);
      nh_private_.getParam("INIT_SETTING/bias_gyro", bias_gyro);
      nh_private_.getParam("INIT_SETTING/bias_accel", bias_acce);
      // convert to eigen
      params.init.setting.orientation << orientation.at(0), orientation.at(1), 
                                         orientation.at(2), orientation.at(3);
      params.init.setting.position << position.at(0), position.at(1), position.at(2);                                         
      params.init.setting.velocity << velocity.at(0), velocity.at(1), velocity.at(2);                                         
      params.init.setting.bias_gyro << bias_gyro.at(0), bias_gyro.at(1), bias_gyro.at(2);                                         
      params.init.setting.bias_acce << bias_acce.at(0), bias_acce.at(1), bias_acce.at(2);                                         

      // --------------- temporal --------------- //
      XmlRpc::XmlRpcValue rosparam_temporal;
      nh_private_.getParam("INIT_SETTING/temporal", rosparam_temporal);
      ROS_ASSERT(rosparam_temporal.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for(uint32_t i = 0 ; i < rosparam_temporal.size() ; i++) {
          for(const auto& name : params.sys.sensors) {
              // convert sensor name enum to string
              auto key = static_cast<std::string>(magic_enum::enum_name(name));

              // check if this sensor time exist
              if(rosparam_temporal[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                  continue;
              }
              // save to parameters
              auto dt = static_cast<double>(rosparam_temporal[i][key]);
              params.init.setting.temporal[name] = dt;
              // each line only has one sensor, so found and break
              break;
          }
      }

      // --------------- Global --------------- //
      XmlRpc::XmlRpcValue rosparam_global;
      nh_private_.getParam("INIT_SETTING/global", rosparam_global);
      ROS_ASSERT(rosparam_global.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for(uint32_t i = 0 ; i < rosparam_global.size() ; i++) {
          for(const auto& name : params.sys.sensors) {
              // convert sensor name enum to string
              auto key = static_cast<std::string>(magic_enum::enum_name(name));
              // check if this sensor time exist
              if(rosparam_global[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                  continue;
              }
              // save to parameters
              for(int j=0; j < rosparam_global[i][key].size(); j++){
                params.init.setting.global[name].push_back(static_cast<double>(rosparam_global[i][key][j]));
              }
              // each line only has one sensor, so found and break
              break;
          }
      }

      for(const auto& kv: params.init.setting.time){
        printf("time: sensor=%s, time=%.9f\n", static_cast<std::string>(magic_enum::enum_name(kv.first)).c_str(), kv.second);
      }

      std::cout<<"orientation(q_x,q_y,q_z,q_w): " << params.init.setting.orientation.transpose() << std::endl;
      std::cout<<"position(x,y,z): " << params.init.setting.position.transpose() <<std::endl;
      std::cout<<"velocity(x,y,z): " << params.init.setting.velocity.transpose() <<std::endl;
      std::cout<<"bias_gyro(x,y,z): " << params.init.setting.bias_gyro.transpose() <<std::endl;
      std::cout<<"bias_acce(x,y,z): " << params.init.setting.bias_acce.transpose() <<std::endl;

      for(const auto& kv: params.init.setting.temporal){
        printf("temporal: sensor=%s, temporal=%.9f\n", static_cast<std::string>(magic_enum::enum_name(kv.first)).c_str(), kv.second);
      }

      for(const auto& kv : params.init.setting.global) {
        for(const auto& vec : params.init.setting.global[kv.first]) {
          printf("global: sensor=%s, value=%f\n", static_cast<std::string>(magic_enum::enum_name(kv.first)).c_str(), vec);
        }
      }

      break;
    }  

    default:
      break;
  }

  /***************************************************************************************/
  /******************************** Priors for each sensor *******************************/
  /***************************************************************************************/

  // ==================== IMU ==================== //
  double gravity;
  nh_private_.param<double>("IMU/gravity",                     gravity,                   9.81);
  nh_private_.param<double>("IMU/accelerometer_noise_density", params.prior_imu.sigma_a,  2.0000e-3);
  nh_private_.param<double>("IMU/accelerometer_random_walk",   params.prior_imu.sigma_ab, 3.0000e-03);
  nh_private_.param<double>("IMU/gyroscope_noise_density",     params.prior_imu.sigma_w,  1.6968e-04);
  nh_private_.param<double>("IMU/gyroscope_random_walk",       params.prior_imu.sigma_wb, 1.9393e-05);
  params.prior_imu.gravity << 0, 0, gravity;

  // ==================== DVL ==================== //
  XmlRpc::XmlRpcValue rosparam_dvl;
  std::vector<double> noise_bt(3);
  //// get DVL extrinsic transformation matrix between IMU and DVl
  nh_private_.getParam     ("DVL/T_I_D",            rosparam_dvl);
  //// get DVL timeoffset
  nh_private_.param<double>("DVL/timeoffset_I_D",   params.prior_dvl.timeoffset, 0.0);
  //// get DVL scale factor 
  nh_private_.param<double>("DVL/scale",            params.prior_dvl.scale, 1.0);
  //// DVL BT velocity measurement noise
  nh_private_.getParam     ("DVL/noise_bt",         noise_bt);

  // ==================== PRESSURE ==================== //

  //// get mount angle
  nh_private_.param<double>("PRESSURE/mount_angle",      params.prior_pressure.mount_angle, 0.0);
  //// pressure measurement noise 
  nh_private_.param<double>("PRESSURE/noise_pressure",   params.prior_pressure.sigma_pressure, 0.0);

  //// convert matrix into pose 
  Eigen::Matrix4d T_I_D;
  ROS_ASSERT(rosparam_dvl.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < rosparam_dvl.size(); ++i) {
    for(int32_t j=0; j<rosparam_dvl[i].size(); ++j) 
      T_I_D(i,j) = static_cast<double>(rosparam_dvl[i][j]);
  }

  params.prior_dvl.extrinsics.block(0, 0, 4, 1) = toQuaternion(T_I_D.block(0, 0, 3, 3));
  params.prior_dvl.extrinsics.block(4, 0, 3, 1) = T_I_D.block(0, 3, 3, 1);
  params.prior_dvl.sigma_bt << noise_bt.at(0), noise_bt.at(1), noise_bt.at(2);
  
  // ==================== Camera ==================== //
  XmlRpc::XmlRpcValue rosparam_cam;
  std::vector<double> distortion_coeffs(4);
  std::vector<double> intrinsics(4);

  nh_private_.getParam     ("CAM0/T_C_I",             rosparam_cam);
  nh_private_.getParam     ("CAM0/distortion_coeffs", distortion_coeffs);
  nh_private_.getParam     ("CAM0/intrinsics",        intrinsics);
  nh_private_.param<double>("CAM0/timeoffset_C_I",    params.prior_cam.timeoffset, 0.0);
  nh_private_.param<double>("CAM0/noise",    params.prior_cam.noise, 1.0);


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


  /***************************************************************************************/
  /********************************** Front-end *******************************/
  /***************************************************************************************/

  // ==================== Image frontend ==================== //
  nh_private_.param<int>   ("KLT/num_aruco",        params.tracking.num_aruco,        1024);
  nh_private_.param<int>   ("KLT/num_pts",          params.tracking.num_pts,          250);
  nh_private_.param<int>   ("KLT/fast_threshold",   params.tracking.fast_threshold,   15);
  nh_private_.param<int>   ("KLT/grid_x",           params.tracking.grid_x,           5);
  nh_private_.param<int>   ("KLT/grid_y",           params.tracking.grid_y,           3);
  nh_private_.param<int>   ("KLT/min_px_dist",      params.tracking.min_px_dist,      8);
  nh_private_.param<bool>  ("KLT/downsize_aruco",   params.tracking.downsize_aruco,   false);
  nh_private_.param<bool>  ("KLT/use_stereo",       params.tracking.use_stereo,       false);
  nh_private_.param<int>   ("KLT/max_camera",       params.tracking.max_camera,       2);
  nh_private_.param<int>   ("KLT/pyram",            params.tracking.pyram,            3);
  nh_private_.param<int>   ("KLT/cam_id",           params.tracking.cam_id,           0);
  nh_private_.param<double>("KLT/downsample_ratio", params.tracking.downsample_ratio, 1.0);
  
  nh_private_.param<double>("Feature/max_cond_number",  params.triangualtion.max_cond_number, 10000);
  nh_private_.param<double>("Feature/min_dist",         params.triangualtion.min_dist,        0.10);
  nh_private_.param<double>("Feature/max_dist",         params.triangualtion.max_dist,        60);
  nh_private_.param<double>("Feature/lam_mult",         params.triangualtion.lam_mult,        10);
  nh_private_.param<int>   ("Feature/max_runs",         params.triangualtion.max_runs,        5);
  nh_private_.param<double>("Feature/max_lamda",        params.triangualtion.max_lamda,       1e10);
  nh_private_.param<double>("Feature/min_dx",           params.triangualtion.min_dx,          1e-6);
  nh_private_.param<double>("Feature/min_dcost",        params.triangualtion.min_dcost,       1e-6);
  nh_private_.param<double>("Feature/max_baseline",     params.triangualtion.max_baseline,    40);

  nh_private_.param<int>   ("Keyframe/frame_count",  params.keyframe.frame_count,  5);
  nh_private_.param<double>("Keyframe/frame_motion", params.keyframe.frame_motion, 0.1);
  nh_private_.param<int>   ("Keyframe/motion_space", params.keyframe.motion_space, 3);
  nh_private_.param<int>   ("Keyframe/min_tracked",  params.keyframe.min_tracked,  50);
  nh_private_.param<double>("Keyframe/scene_ratio",  params.keyframe.scene_ratio,  0.8);

  // ==================== MSCKF ==================== //
  nh_private_.param<bool>("MSCKF/dvl_exterisic_R", params.msckf.do_R_I_D,    true);
  nh_private_.param<bool>("MSCKF/dvl_exterisic_p", params.msckf.do_p_I_D,    true);
  nh_private_.param<bool>("MSCKF/dvl_timeoffset",  params.msckf.do_time_I_D, true);
  nh_private_.param<bool>("MSCKF/dvl_scale",       params.msckf.do_scale_D,  true);
  nh_private_.param<int> ("MSCKF/dvl_clone",       params.msckf.max_clone_D, 2);

  nh_private_.param<bool>("MSCKF/cam_exterisic_R", params.msckf.do_R_C_I,    true);
  nh_private_.param<bool>("MSCKF/cam_exterisic_p", params.msckf.do_p_C_I,    true);
  nh_private_.param<bool>("MSCKF/cam_timeoffset",  params.msckf.do_time_C_I, true);
  nh_private_.param<int> ("MSCKF/cam_clone",       params.msckf.max_clone_C, 9);

  nh_private_.param<int> ("MSCKF/max_msckf_update", params.msckf.max_msckf_update, params.tracking.num_pts);

  nh_private_.getParam("MSCKF/marginalized_clone", params.msckf.marginalized_clone);
  
  return params;
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
  message.a << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  message.w << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

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
  cv::Mat img;
  int width = cv_ptr->image.cols * parameters.tracking.downsample_ratio;
  int height = cv_ptr->image.rows * parameters.tracking.downsample_ratio;
  cv::resize(cv_ptr->image, img, cv::Size(width, height));
  //! TODO: cv::resize vs. cv::pyrDown

  // feed img
  ImageMsg message;
  message.image = img;
  message.time = msg->header.stamp.toSec();;

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

void RosNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  //! TODO: multi-path will has degraded measurement, filter multi-path based on sub_map, plane-constrain
  // pcl::PCLPointCloud2 pc2;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl_conversions::toPCL(*msg, pc2);
  // pcl::fromPCLPointCloud2(pc2, *cloud);

}

void RosNode::process() {
  
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