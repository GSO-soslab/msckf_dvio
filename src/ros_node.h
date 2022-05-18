#ifndef MSCKF_ROS_NODE_H_
#define MSCKF_ROS_NODE_H_

//c++
#include <fstream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// #include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nortek_dvl/ButtomTrack.h>

//
#include "core/msckf_manager.h"
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

  void dvlCallback(const nortek_dvl::ButtomTrack::ConstPtr &msg);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void process();

  Params loadParameters();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber dvl_sub_;
  ros::Subscriber image_sub_;

  std::shared_ptr<MsckfManager> manager;

  Params parameters;

}; // end of class   

} // namespace msckf_dvio

#endif  //MSCKF_ROS_NODE_H_