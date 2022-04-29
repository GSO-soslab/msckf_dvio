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
  nh_private_.param<double>("IMU/gravity", params.gravity, 9.81);

  //===== DVL =====//
  //// get DVL extrinsic transformation matrix between IMU and DVl
  XmlRpc::XmlRpcValue ros_param_list;
  nh_private_.getParam("DVL/T_I_D", ros_param_list);
  //// get DVL timeoffset
  nh_private_.param<double>("DVL/timeoffset_I_D", params.timeoffset_I_D, 0.0);
  //// get DVL scale factor
  nh_private_.param<double>("DVL/scale", params.scale, 0.0);

  //// convert matrix into pose (q_I_D, p_I_D)
  Eigen::Matrix4d T_I_D;
  ROS_ASSERT(ros_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < ros_param_list.size(); ++i) {
    for(int32_t j=0; j<ros_param_list[i].size(); ++j) {
      T_I_D(i,j) = static_cast<double>(ros_param_list[i][j]);
    }
  }

  Eigen::Matrix<double, 7, 1> dvl_extrinsics;
  dvl_extrinsics.block(0, 0, 4, 1) = toQuaternion(T_I_D.block(0, 0, 3, 3));
  dvl_extrinsics.block(4, 0, 3, 1) = T_I_D.block(0, 3, 3, 1);
  params.dvl_extrinsics = dvl_extrinsics;

  //===== Camera =====//
  // nh_private_.param<double>("Camera/timeoffset_I_C", params.timeoffset_I_C, 0.0);

/******************** System ********************/
  nh_private_.param<int>("SYS/backend_hz", params.backend_hz, 20);
  nh_private_.param<int>("SYS/imu_init_mode", params.imu_init_mode, 1);
  nh_private_.param<int>("SYS/imu_windows", params.imu_windows, 20);
  nh_private_.param<double>("SYS/imu_var", params.imu_var, 0.2);
  nh_private_.param<double>("SYS/imu_delta", params.imu_delta, 0.07);
  nh_private_.param<int>("SYS/dvl_windows", params.dvl_windows, 4);
  nh_private_.param<double>("SYS/dvl_delta", params.dvl_delta, 0.05);

  return params;
}

void RosNode::imuCallback(const sensor_msgs::ImuConstPtr &msg) {

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