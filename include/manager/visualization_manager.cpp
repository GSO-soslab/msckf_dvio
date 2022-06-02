#include "visualization_manager.h"

namespace msckf_dvio
{

VisualizationManager::VisualizationManager(ros::NodeHandle &nh, std::shared_ptr<MsckfManager> msckf_manager):
  msckf_manager_(msckf_manager)
{
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  pub_path = nh.advertise<nav_msgs::Path>("/path", 10);
}

} // end of namespace msckf_dvio