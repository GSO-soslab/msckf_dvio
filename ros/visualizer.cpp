#include "visualizer.h"

namespace msckf_dvio
{

RosVisualizer::RosVisualizer(const ros::NodeHandle &nh, std::shared_ptr<MsckfManager> manager)
  :msckf_manager(manager), it_(nh), nh_(nh), last_visual_times(0) {

  // Setup our transform broadcaster
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();

  // Setup ros sub/pub
  pub_img = it_.advertise("/tracked_img", 20);
  pub_odom = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
  pub_path = nh_.advertise<nav_msgs::Path>("/path", 10);
  pub_features = nh_.advertise<sensor_msgs::PointCloud2>("/feature_clouds", 10); 

}

void RosVisualizer::visualize() {
  // Return if we have already visualized
  if (last_visual_times == msckf_manager->getState()->getTimestamp()) {
    return;
  }
  last_visual_times = msckf_manager->getState()->getTimestamp();

  // publish current image
  publishImage();

  // return if system is not initialized
  if(!msckf_manager->isInitialized()) {
    return;
  } 

  // publish State
  publishState();

  // publish triangulated features
  publishFeatures();

}

void RosVisualizer::publishImage() {
  // Check if we have subscribers
  if (pub_img.getNumSubscribers() == 0)
    return;

  // Get our image of history tracks
  cv::Mat img_history;
  msckf_manager->getTracker()->display_history(img_history, 0, 255, 255, 255, 255, 255);

  // create ROS image message
  std_msgs::Header header;
  header.frame_id = "img";
  header.stamp = ros::Time::now();
  sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(header, "bgr8", img_history).toImageMsg();

  // Publish
  pub_img.publish(msg_img);
}

void RosVisualizer::publishState() {
  //// get staic tf: Base_link ~ AHRS
  tf2::Quaternion q_B_I;
  q_B_I.setRPY( 3.132, 0.003, 3.130);
  tf2::Vector3 p_B_I(0.295, 0.083, 0.090);
  tf2::Transform T_B_I;
  T_B_I.setRotation(q_B_I);
  T_B_I.setOrigin(p_B_I);

  //// get tf: Odometry ~ AHRS
  auto imu_value = msckf_manager->getState()->getImuValue();
  auto imu_time = msckf_manager->getState()->getTimestamp();

  tf2::Transform T_O_I;
  T_O_I.setOrigin( tf2::Vector3(imu_value(4), imu_value(5), imu_value(6)) );
  T_O_I.setRotation( tf2::Quaternion(imu_value(0),imu_value(1),imu_value(2),imu_value(3)) );

  //// get tf: Odometry ~ Base_link
  tf2::Transform T_O_B = T_O_I * T_B_I.inverse();

  //// Publish tf
  geometry_msgs::TransformStamped tf_O_B;
  tf2::convert(T_O_B, tf_O_B.transform);

  tf_O_B.header.stamp = ros::Time(imu_time);
  tf_O_B.header.frame_id = "odom";
  tf_O_B.child_frame_id = "base_link";
  odom_broadcaster->sendTransform(tf_O_B);

  //// Publish odometry

  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
  nav_msgs::Odometry msg_odom;

  msg_odom.header.stamp = ros::Time(imu_time);
  msg_odom.header.frame_id = "odom";
  msg_odom.child_frame_id = "ahrs";
  msg_odom.pose.pose.orientation.x = imu_value(0);
  msg_odom.pose.pose.orientation.y = imu_value(1);
  msg_odom.pose.pose.orientation.z = imu_value(2);
  msg_odom.pose.pose.orientation.w = imu_value(3);
  msg_odom.pose.pose.position.x    = imu_value(4);
  msg_odom.pose.pose.position.y    = imu_value(5);
  msg_odom.pose.pose.position.z    = imu_value(6);
  msg_odom.twist.twist.linear.x    = imu_value(7);
  msg_odom.twist.twist.linear.y    = imu_value(8);
  msg_odom.twist.twist.linear.z    = imu_value(9);

  pub_odom.publish(msg_odom);

  //// publish path

  geometry_msgs::PoseStamped pose;

  pose.header.frame_id = msg_odom.header.frame_id;
  pose.header.stamp = msg_odom.header.stamp;
  pose.pose = msg_odom.pose.pose;

  path.header.frame_id = msg_odom.header.frame_id;
  path.header.stamp = msg_odom.header.stamp;
  path.poses.push_back(pose);

  pub_path.publish(path);
  

  // tf::StampedTransform trans;
  // // trans.stamp_ = ros::Time::now();
  // trans.stamp_ = ros::Time(imu_time);
  // trans.frame_id_ = "odom";
  // trans.child_frame_id_ = "imu";
  // tf::Quaternion quat(msg_odom.pose.pose.orientation.x, 
  //                     msg_odom.pose.pose.orientation.y, 
  //                     msg_odom.pose.pose.orientation.z, 
  //                     msg_odom.pose.pose.orientation.w);
  // trans.setRotation(quat);
  // tf::Vector3 orig(msg_odom.pose.pose.position.x, 
  //                  msg_odom.pose.pose.position.y, 
  //                  msg_odom.pose.pose.position.z);
  // trans.setOrigin(orig);
  // odom_broadcaster->sendTransform(trans);

}

void RosVisualizer::publishFeatures() {
// visualize tracked feature in 3D

  // get features
  std::vector<Eigen::Vector3d> feats = msckf_manager->getFeatures();

  if(feats.size() == 0 ) {
    return;
  }
  
  // setup the points XYZ
  sensor_msgs::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");    
  modifier.resize(feats.size()); 
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(cloud_msg, "z");

  // copy to msg
  for (size_t i = 0; i < feats.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
      const Eigen::Vector3d& point = feats.at(i);
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
  }

  // publish
  cloud_msg.header.frame_id = "odom";
  cloud_msg.header.stamp = ros::Time::now();
  pub_features.publish(cloud_msg);

}

} // end of namespace msckf_dvio