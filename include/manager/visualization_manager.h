#ifndef MSCKF_MANAGER_VISUALIZATION_H
#define MSCKF_MANAGER_VISUALIZATION_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include "msckf_manager.h"

namespace msckf_dvio
{

class VisualizationManager {
public:

  //! @brief Default constructor
  //! @param nh ROS node handler
  //! @param app Core estimator manager
  VisualizationManager(ros::NodeHandle &nh, std::shared_ptr<MsckfManager> msckf_manager);

private:

  std::shared_ptr<MsckfManager> msckf_manager_;

  ros::Publisher pub_odom, pub_path;

};

} // end of namespace msckf_dvio

#endif // MSCKF_MANAGER_VISUALIZATION_H