#ifndef MSCKF_ROS_VISUALIZER_H
#define MSCKF_ROS_VISUALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "manager/msckf_manager.h"

namespace msckf_dvio
{

class RosVisualizer {
public:

  /**
   * @brief Default constructor
   * 
   * @param nh ROS node handler
   * @param app Core estimator manager
   */
  RosVisualizer(const ros::NodeHandle &nh, std::shared_ptr<MsckfManager> manager);

  /**
   * @brief Will visualize the system if we have new things
   */
  void visualize();

private:
  /// Publish the current state
  void publishState();

  /// Publish the active tracking image
  void publishImage();

  /// Publish current features
  void publishFeatures();

  std::shared_ptr<MsckfManager> msckf_manager;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::Publisher pub_odom, pub_path;
  ros::Publisher pub_features;
  image_transport::Publisher pub_img;

  nav_msgs::Path path;

  std::unique_ptr<tf::TransformBroadcaster> odom_broadcaster;

  double last_visual_times;
  
};

} // end of namespace msckf_dvio

#endif // MSCKF_ROS_VISUALIZER_H