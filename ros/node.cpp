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
  //// get mount angle
  nh_private_.param<double>("DVL/mount_angle",      params.prior_dvl.mount_angle, 0.0);
  //// DVL pressure measurement noise 
  nh_private_.param<double>("DVL/noise_pressure",   params.prior_dvl.sigma_pressure, 0.0);

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
/********************************** System configuration *******************************/
/***************************************************************************************/

  // ==================== System ==================== //
  nh_private_.param<int>   ("SYS/backend_hz",      params.backend_hz,      20);
  nh_private_.param<double>("SYS/dvl_v_threshold", params.dvl_v_threshold, 2.0);

  // ==================== Initialization ==================== //
  int init_mode;

  nh_private_.param<int>   ("INIT/mode",     init_mode,     2);
  nh_private_.param<int>   ("INIT/imu_init_mode",     params.init.imu_init_mode,     1);
  nh_private_.param<int>   ("INIT/imu_window",        params.init.imu_window,        20);
  nh_private_.param<double>("INIT/imu_var",           params.init.imu_var,           0.2);
  nh_private_.param<double>("INIT/imu_delta",         params.init.imu_delta,         0.07);
  nh_private_.param<int>   ("INIT/dvl_window",        params.init.dvl_window,        4);
  nh_private_.param<double>("INIT/dvl_delta",         params.init.dvl_delta,         0.05);
  nh_private_.param<double>("INIT/dvl_delta",         params.init.dvl_delta,         0.05);
  nh_private_.param<double>("INIT/dvl_init_duration", params.init.dvl_init_duration, 1.0);
  nh_private_.param<bool>  ("INIT/init_given",    params.init.init_given,    false);

  // convert 

  params.init.mode = static_cast<InitMode>(init_mode);

  std::vector<double> init_state(17);
  if(params.init.init_given ) {
    nh_private_.getParam   ("INIT/init_state",    init_state);

    params.init.init_state << init_state.at(0), //t
                              init_state.at(1),init_state.at(2),init_state.at(3),init_state.at(4), //q
                              init_state.at(5),init_state.at(6),init_state.at(7), //p
                              init_state.at(8),init_state.at(9),init_state.at(10), //v
                              init_state.at(11),init_state.at(12),init_state.at(13), //bg
                              init_state.at(14),init_state.at(15),init_state.at(16); //ba
  }

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
  
  int sleep_t = 1.0 / parameters.backend_hz * 1000.0;

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