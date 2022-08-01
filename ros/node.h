#ifndef MSCKF_ROS_NODE_H_
#define MSCKF_ROS_NODE_H_

//c++
#include <fstream>
#include <atomic>
#include <mutex>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/FluidPressure.h>  
#include <sensor_msgs/PointCloud2.h>  
#include <nortek_dvl/ButtomTrack.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>

//
#include "manager/msckf_manager.h"
#include "types/type_all.h"

#include "visualizer.h"

namespace msckf_dvio
{

class RosNode
{
public:
  RosNode(const ros::NodeHandle &nh,
          const ros::NodeHandle &nh_private);

  ~RosNode(){}

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  void dvlCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg);

  void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

  void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr &msg);

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  bool srvCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

  void process();

  Params loadParameters();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer service_;

  ros::Subscriber sub_imu;
  ros::Subscriber sub_dvl;
  ros::Subscriber sub_img;
  ros::Subscriber sub_pressure;
  ros::Subscriber sub_pointcloud;

  double last_t_img = 0;
  double last_t_dvl = 0;
  double last_t_pressure = 0;
  double last_t_pointcloud = 0;

  std::shared_ptr<MsckfManager> manager;

  std::shared_ptr<RosVisualizer> visualizer;

  Params parameters;

  //! TEST: visualization

  ros::Publisher pub_features;



}; // end of class   

} // namespace msckf_dvio

#endif  //MSCKF_ROS_NODE_H_