#include "node.h"

#include <thread>

namespace msckf_dvio
{

RosNode::RosNode(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private), it_(nh)
{
  // get parameters and feed to manager system
  parameters = loadParameters();

  manager = std::make_shared<MsckfManager>(parameters);

  // ROS related
  sub_imu = nh_.subscribe("imu", 2000, &RosNode::imuCallback, this);
  sub_dvl = nh_.subscribe("dvl", 100, &RosNode::dvlCallback, this);
  sub_img = nh_.subscribe("image", 200, &RosNode::imageCallback, this);
  sub_pressure = nh_.subscribe("pressure", 100, &RosNode::pressureCallback, this);
  sub_pointcloud = nh_.subscribe("pointcloud", 100, &RosNode::pointcloudCallback, this);

  //! TEST:
  pub_odom = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
  pub_path = nh_.advertise<nav_msgs::Path>("/path", 10);  

  service_ = nh_.advertiseService("cmd",&RosNode::srvCallback, this);

  odom_broadcaster = new tf::TransformBroadcaster();

  pub_img_1 = it_.advertise("/tracked_img1", 20);

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
  nh_private_.param<int>   ("INIT/imu_init_mode",     params.init.imu_init_mode,     1);
  nh_private_.param<int>   ("INIT/imu_window",        params.init.imu_window,        20);
  nh_private_.param<double>("INIT/imu_var",           params.init.imu_var,           0.2);
  nh_private_.param<double>("INIT/imu_delta",         params.init.imu_delta,         0.07);
  nh_private_.param<int>   ("INIT/dvl_window",        params.init.dvl_window,        4);
  nh_private_.param<double>("INIT/dvl_delta",         params.init.dvl_delta,         0.05);
  nh_private_.param<double>("INIT/dvl_delta",         params.init.dvl_delta,         0.05);
  nh_private_.param<double>("INIT/dvl_init_duration", params.init.dvl_init_duration, 1.0);
  
  nh_private_.param<bool>  ("INIT/init_given",    params.init.init_given,    false);
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

  // ==================== Tracking ==================== //
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

}

void RosNode::process() {
  
  int sleep_t = 1.0 / parameters.backend_hz * 1000.0;

  while(1) {
    // do the ekf stuff
    manager->backend();

    //! TODO: move this to visulization manager
    // get imu state to publish 
    if(manager->isOdom()) {

      auto imu_value = manager->getNewImuState();
      auto time = manager->getTime();
      manager->resetOdom();

      //! TEST:
      //// JPL form
      // Eigen::Vector4d q_JPL_I_G;
      // q_JPL_I_G << imu_value(0), imu_value(1), imu_value(2), imu_value(3);
      // Eigen::Matrix3d R_I_G = toRotationMatrix(q_JPL_I_G);
      // std::cout<<"JPL x: "<<q_JPL_I_G(0)<<" y: "<< q_JPL_I_G(1)<< " z: "<<q_JPL_I_G(2)<< " w: "<< q_JPL_I_G(3)<<std::endl;
      //// Hamilton form
      // Eigen::Matrix3d R_G_I = R_I_G.transpose();
      // Eigen::Quaterniond q_Ham_G_I(R_G_I);
      // q_Ham_G_I.normalize();
      // std::cout<<"Hamilton x: "<<q_Ham_G_I.x()<<" y: "<<q_Ham_G_I.y()<<" z: "<<q_Ham_G_I.z()<<" w: "<<q_Ham_G_I.w()<<std::endl;

      //// start:
      // JPL x: -0.992789 y: 0.0683307 z: 0.0790672 w: 0.0587278
      // Hamilton x: 0.992789 y: -0.0683307 z: -0.0790672 w: -0.0587278
      //// after turning round, it's wrong ???
      // JPL x: -0.170721 y: 0.982622 z: -0.051178 w: 0.0518503
      // Hamilton x: -0.170721 y: 0.982622 z: -0.051178 w: 0.0518503

      //// publish odometry
      nav_msgs::Odometry msg_odom;
      msg_odom.header.stamp = ros::Time(time);
      msg_odom.header.frame_id = "odom";
      msg_odom.child_frame_id = "imu";
      // msg_odom.pose.pose.orientation.x = q_Ham_G_I.x();
      // msg_odom.pose.pose.orientation.y = q_Ham_G_I.y();
      // msg_odom.pose.pose.orientation.z = q_Ham_G_I.z();
      // msg_odom.pose.pose.orientation.w = q_Ham_G_I.w();
      msg_odom.pose.pose.orientation.x = imu_value(0);
      msg_odom.pose.pose.orientation.y = imu_value(1);
      msg_odom.pose.pose.orientation.z = imu_value(2);
      msg_odom.pose.pose.orientation.w = imu_value(3);
      msg_odom.pose.pose.position.x    = imu_value(4);
      msg_odom.pose.pose.position.y    = imu_value(5);
      msg_odom.pose.pose.position.z    = imu_value(6);
      msg_odom.twist.twist.linear.x    = imu_value(7);
      msg_odom.twist.twist.linear.y    = imu_value(8);
      msg_odom.twist.twist.linear.z    = imu_value(9);

      pub_odom.publish(msg_odom);

      // Publish odometry TF
      // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
      // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
      tf::StampedTransform trans;
      trans.stamp_ = ros::Time::now();
      trans.frame_id_ = "odom";
      trans.child_frame_id_ = "imu";
      tf::Quaternion quat(msg_odom.pose.pose.orientation.x, msg_odom.pose.pose.orientation.y, 
                          msg_odom.pose.pose.orientation.z, msg_odom.pose.pose.orientation.w);
      trans.setRotation(quat);
      tf::Vector3 orig(imu_value(4), imu_value(5), imu_value(6));
      trans.setOrigin(orig);
      odom_broadcaster->sendTransform(trans);

      //// Publish path
      geometry_msgs::PoseStamped pose;

      pose.header.frame_id = msg_odom.header.frame_id;
      pose.header.stamp = msg_odom.header.stamp;
      pose.pose = msg_odom.pose.pose;

      path.header.frame_id = msg_odom.header.frame_id;
      path.header.stamp = msg_odom.header.stamp;
      path.poses.push_back(pose);

      pub_path.publish(path);
    }

    // visualize tracked features
    if(manager->checkTrackedImg()) {
      ImageMsg img = manager->getTrackedImg();

      std_msgs::Header header;
      header.frame_id = "img";
      header.stamp = ros::Time(img.time);
      sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(header, "bgr8", img.image).toImageMsg();
      pub_img_1.publish(msg_img);
    }

    std::chrono::milliseconds dura(sleep_t);
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

  // backendThread.join();

  return 0;
}