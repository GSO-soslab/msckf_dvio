// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
// c++
#include <memory>
#include <vector>
#include <tuple>
#include <iostream>
// Eigen
#include <Eigen/Eigen>
// self
#include "utils/utils.h"

////////////////////////////////////////////////////////////////////////
// rosrun msckf_dvio test_imu_init_rosbag _bag_start:=0 _bag_durr:=2
///////////////////////////////////////////////////////////////////////

std::tuple<Eigen::Vector3d, Eigen::Vector3d>
process(const std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> &data)
{
  Eigen::Vector3d gyro_mean, acce_mean;
  for(const auto &entry : data) {

    gyro_mean += std::get<1>(entry);
    acce_mean += std::get<2>(entry);
  }

  gyro_mean = gyro_mean / data.size();
  acce_mean = acce_mean / data.size();

  //// estimate gyro bias
  Eigen::Vector3d bias_gyro = gyro_mean;

  //// Normalize the gravity z-axis that projected into IMU frame:
  ////   cosine of inertial frame z-axis (gravity align with z-axis) with IMU frame's x-axis,y-axis and x-axis
  Eigen::Vector3d z_I_G = acce_mean / acce_mean.norm();
  
  //// Normalize the gravity x-axis that projected into IMU frame:
  ////    Get x-axis to perpendicular to z-axis
  ////    Use [Gram-Schmidt Process](https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
  Eigen::Vector3d x_I(1, 0, 0);
  Eigen::Vector3d x_I_G = x_I - z_I_G * z_I_G.transpose() * x_I;
  x_I_G = x_I_G / x_I_G.norm();

  //// Normalize the gravity y-axis that projected into IMU frame:
  ////    Get y from the cross product of these two
  Eigen::Vector3d y_I_G = msckf_dvio::toSkewSymmetric(z_I_G) * x_I_G;

  // From these axes get rotation
  Eigen::Matrix3d R_I_G;
  R_I_G.block(0, 0, 3, 1) = x_I_G;
  R_I_G.block(0, 1, 3, 1) = y_I_G;
  R_I_G.block(0, 2, 3, 1) = z_I_G;

  //// estimate acceleration bias
  Eigen::Vector3d bias_acce = acce_mean - R_I_G * Eigen::Vector3d(0.0,0.0,9.81);

  return std::make_tuple(bias_gyro, bias_acce); 
}

// Main function
int main(int argc, char **argv) {

  // Launch our ros node
  ros::init(argc, argv, "rosbag_example");
  ros::NodeHandle nh("~");

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our imu topic
  std::string topic_imu;
  nh.param<std::string>("topic_imu", topic_imu, "/rov/sensors/ahrs/imu/data");
  ROS_INFO("read topic is: %s", topic_imu.c_str());

  // Location of the ROS bag we want to read in
  std::string path_to_bag;
  nh.param<std::string>("path_bag", path_to_bag, 
                        "/home/lin/develop/data/underIce/GLRC/calib/ahrs/noise_calibration/for_imu_kaliba_allen.bag");
  ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, 5);
  ROS_INFO("bag start: %.1f", bag_start);
  ROS_INFO("bag duration: %.1f", bag_durr);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Load rosbag here, and find messages we can play
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  // We should load the bag as a view
  // Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  ROS_INFO("time start = %.6f", time_init.toSec());
  ROS_INFO("time end   = %.6f", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Step through the rosbag
  std::vector<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> imu_data;
  Eigen::Vector3d gyro, acce;
  double timestamp;

  for (const rosbag::MessageInstance &m : view) {

    // If ros is wants us to stop, break out
    if (!ros::ok())
      break;

    // Grab IMU measurement
    sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
    if (imu != nullptr && m.getTopic() == topic_imu) {
      timestamp = (*imu).header.stamp.toSec();
      gyro << (*imu).angular_velocity.x, (*imu).angular_velocity.y, (*imu).angular_velocity.z;
      acce << (*imu).linear_acceleration.x, (*imu).linear_acceleration.y, (*imu).linear_acceleration.z;

      imu_data.emplace_back(timestamp, gyro, acce);
    }
  }

  // Process this grabbed data
  Eigen::Vector3d b_g, b_a;
  std::tie(b_g, b_a) = process(imu_data);
  std::cout<<"data size: "<<imu_data.size()<<"view size: "<< view.size()<<"\n";

  std::cout<<"b_g: \n"<< b_g<<"\nb_a: \n"<< b_a<<"\n";

  // Done!
  return EXIT_SUCCESS;
}


///////////////////// beginning //////////////////////
// b_g: 
//  -0.00095975
//   0.00122211
// -2.48293e-06
// b_a: 
// -1.01283e-05
//  7.21916e-06
//  0.000832314
///////////////////// 0.5 hours:  //////////////////////
// b_g: 
// -0.000985325
//    0.0020182
// -3.02397e-05
// b_a: 
// -3.73773e-05
//  2.69838e-05
//   0.00321798

///////////////////// 1 hours:  //////////////////////
// b_g: 
//   -0.0011269
//   0.00252905
// -0.000197337
// b_a: 
// -5.18209e-05
//  3.64892e-05
//   0.00453614

///////////////////// 1.5 hours:  //////////////////////
// b_g: 
//  -0.00159715
//   0.00267193
// -0.000136397
// b_a: 
// -6.00215e-05
//   4.1805e-05
//   0.00531994

///////////////////// 2 hours:  //////////////////////
// b_g: 
//  -0.00169236
//   0.00285331
// -0.000348906
// b_a: 
// -7.16947e-05
//   4.9279e-05
//    0.0063694

///////////////////// 2.5 hours:  //////////////////////
// b_g: 
//  -0.00169639
//   0.00300457
// -0.000454593
// b_a: 
// -8.16951e-05
//  5.55377e-05
//   0.00728187

///////////////////// 3 hours:  //////////////////////
// b_g: 
//  -0.00210342
//   0.00289581
// -0.000455431
// b_a: 
// -8.68381e-05
//  5.87657e-05
//   0.00774041

///////////////////// 3.5 hours:  //////////////////////
// b_g: 
//  -0.00197927
//   0.00297025
// -0.000442401
// b_a: 
// -8.80884e-05
//  5.92716e-05
//   0.00788321

///////////////////// 4 hours:  //////////////////////
