#include "ros_node.h"

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

  // ROS related
  imu_sub_ = nh_.subscribe("imu", 2000, &RosNode::imuCallback, this);
  dvl_sub_ = nh_.subscribe("dvl", 200, &RosNode::dvlCallback, this);
  image_sub_ = nh_.subscribe("image", 200, &RosNode::imageCallback, this);
}    

Params RosNode::loadParameters() {

  Params params;

/******************** State ********************/
  //===== IMU =====//
  double gravity;
  nh_private_.param<double>("IMU/gravity",                     gravity,                   9.81);
  nh_private_.param<double>("IMU/accelerometer_noise_density", params.prior_imu.sigma_a,  2.0000e-3);
  nh_private_.param<double>("IMU/accelerometer_random_walk",   params.prior_imu.sigma_ab, 3.0000e-03);
  nh_private_.param<double>("IMU/gyroscope_noise_density",     params.prior_imu.sigma_w,  1.6968e-04);
  nh_private_.param<double>("IMU/gyroscope_random_walk",       params.prior_imu.sigma_wb, 1.9393e-05);
  params.prior_imu.gravity << 0, 0, gravity;

  //===== DVL =====//
  XmlRpc::XmlRpcValue rosparam_T;
  std::vector<double> bt_noise_density(3);
  //// get DVL extrinsic transformation matrix between IMU and DVl
  nh_private_.getParam     ("DVL/T_I_D",            rosparam_T);
  //// get DVL timeoffset
  nh_private_.param<double>("DVL/timeoffset_I_D",   params.prior_dvl.timeoffset, 0.0);
  //// get DVL scale factor 
  nh_private_.param<double>("DVL/scale",            params.prior_dvl.scale, 1.0);
  //// get DVL BT 3-axis velocity measurement noise
  nh_private_.getParam     ("DVL/bt_noise_density", bt_noise_density);
  
  //// convert matrix into pose 
  Eigen::Matrix4d T;
  ROS_ASSERT(rosparam_T.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < rosparam_T.size(); ++i) {
    for(int32_t j=0; j<rosparam_T[i].size(); ++j) 
      T(i,j) = static_cast<double>(rosparam_T[i][j]);
  }

  params.prior_dvl.extrinsics.block(0, 0, 4, 1) = toQuaternion(T.block(0, 0, 3, 3));
  params.prior_dvl.extrinsics.block(4, 0, 3, 1) = T.block(0, 3, 3, 1);
  params.prior_dvl.sigma_bt << bt_noise_density.at(0), bt_noise_density.at(1), bt_noise_density.at(2);

  
  //===== Camera =====//
  // nh_private_.param<double>("Camera/timeoffset_I_C", params.timeoffset_I_C, 0.0);

/******************** System ********************/
  nh_private_.param<int>("SYS/backend_hz", params.backend_hz, 20);

  nh_private_.param<int>   ("INIT/imu_init_mode", params.init.imu_init_mode, 1);
  nh_private_.param<int>   ("INIT/imu_window",    params.init.imu_window,   20);
  nh_private_.param<double>("INIT/imu_var",       params.init.imu_var,       0.2);
  nh_private_.param<double>("INIT/imu_delta",     params.init.imu_delta,     0.07);
  nh_private_.param<int>   ("INIT/dvl_window",    params.init.dvl_window,   4);
  nh_private_.param<double>("INIT/dvl_delta",     params.init.dvl_delta,     0.05);

  nh_private_.param<bool>("MSCKF/dvl_exterisic_R", params.msckf.do_R_I_D,    true);
  nh_private_.param<bool>("MSCKF/dvl_exterisic_p", params.msckf.do_p_I_D,    true);
  nh_private_.param<bool>("MSCKF/dvl_timeoffset",  params.msckf.do_time_I_D, true);
  nh_private_.param<bool>("MSCKF/dvl_scale",       params.msckf.do_scale_D,  true);
  nh_private_.param<int> ("MSCKF/dvl_clone",       params.msckf.max_clone_D, 2);

  return params;
}

void RosNode::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  //! NOTE: IMU timestamp is not stable: 100hz, actually is duraction is about 0.11,0.11,0.6,0.11,0.11,0.6

  ImuMsg message;
  message.time = msg->header.stamp.toSec();
  message.a << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  message.w << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  manager->feedImu(message);
}

void RosNode::dvlCallback(const nortek_dvl::ButtomTrack::ConstPtr &msg) {
  DvlMsg message;
  message.time = msg->header.stamp.toSec();
  message.v << msg->speed.x, msg->speed.y, msg->speed.z;

  manager->feedDvl(message); 
}


// TODO: check if feature tracking in image callback will effect IMU callback(overflow, bad imu-image align)
void RosNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

  cv_bridge::CvImagePtr cv_ptr;
  try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  ImageMsg message;
  message.image = cv_ptr->image;
  message.time = msg->header.stamp.toSec();;

  manager->feedCamera(message);
}

void RosNode::process() {
  
  int sleep_t = 1.0/parameters.backend_hz *1000.0;


  while(1) {
    // do the ekf stuff
    manager->backend();

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