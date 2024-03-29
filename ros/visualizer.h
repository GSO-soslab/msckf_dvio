#ifndef MSCKF_ROS_VISUALIZER_H
#define MSCKF_ROS_VISUALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "manager/msckf_manager.h"
#include <std_srvs/Trigger.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <atomic>

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

  /// Publish the active tracking image
  void publishImage();

  bool srvCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

private:

  /// Publish the current state
  void publishState();

  /// Publish current features
  void publishFeatures();

  void publishPointCloud();

  std::shared_ptr<MsckfManager> msckf_manager;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::ServiceServer service_;

  ros::Publisher pub_odom, pub_path, pub_bias, pub_pc;
  ros::Publisher pub_features;
  image_transport::Publisher pub_img;

  nav_msgs::Path path;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

  double last_visual_times;

  tf2_ros::Buffer transform_buffer;
  
  Eigen::Isometry3d T_B_I;

  pcl::PointCloud<pcl::PointXYZRGB> saved_pcd;

  std::atomic<bool> save_flag;
};

} // end of namespace msckf_dvio

#endif // MSCKF_ROS_VISUALIZER_H