#ifndef MSCKF_ROS_NODE_H_
#define MSCKF_ROS_NODE_H_

//c++
#include <fstream>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
//
#include "core/msckf_manager.h"
#include "types/type.h"


namespace msckf_dvio
{

class RosNode
{
public:
  RosNode(ros::NodeHandle nh);

  ~RosNode(){}

  void imuCallback(const sensor_msgs::ImuConstPtr &msg);

  void dvlCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void process();

  Params loadParameters();

private:
  ros::NodeHandle nh_;

  ros::Subscriber imu_sub_;
  ros::Subscriber dvl_sub_;
  ros::Subscriber image_sub_;

  std::shared_ptr<MsckfManager> manager;

  Params parameters;

  /******************** remap dvl time ********************/
  bool mapDvlTime(const DvlMsg &in);

  std::vector<std::tuple<Eigen::Vector3d, double, double>> remap_queue;
  std::vector<DvlMsg> remapped_queue;
  double last_integral = 0.0;
  std::string last_flag ="#";

  const char *file_path="/home/lin/Desktop/remap_dvl_time.dat";
  std::ofstream file;

}; // end of class   

} // namespace msckf_dvio

#endif  //MSCKF_ROS_NODE_H_