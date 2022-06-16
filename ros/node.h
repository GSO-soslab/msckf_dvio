#ifndef MSCKF_ROS_NODE_H_
#define MSCKF_ROS_NODE_H_

//c++
#include <fstream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nortek_dvl/ButtomTrack.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
//
#include "manager/msckf_manager.h"
#include "types/type_all.h"


namespace msckf_dvio
{

class RosNode
{
public:
  RosNode(const ros::NodeHandle &nh,
          const ros::NodeHandle &nh_private);

  ~RosNode(){}

  void imuCallback(const sensor_msgs::ImuConstPtr &msg);

  void dvlCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg);

  // void dvlCallback(const nortek_dvl::ButtomTrack::ConstPtr &msg);

  // void dvlCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  bool srvCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

  void process();

  Params loadParameters();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber dvl_sub_;
  ros::Subscriber image_sub_;

  ros::ServiceServer service_;

  std::shared_ptr<MsckfManager> manager;

  Params parameters;

  //! TEST:
  ros::Publisher pub_odom, pub_path;
  tf::TransformBroadcaster *odom_broadcaster;
  nav_msgs::Path path;
}; // end of class   

} // namespace msckf_dvio

#endif  //MSCKF_ROS_NODE_H_