#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>

#include <sensor_msgs/PointCloud.h>


void callback(const sensor_msgs::PointCloud::ConstPtr &msg);


int main(int argc, char **argv) {

  // Launch our ros node
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  // Create msg subscriber
  ros::Subscriber sub = nh.subscribe("/ov_msckf/feature_sim", 100, callback);

  ROS_INFO("done...spinning to ros");
  ros::spin();
}

void callback(const sensor_msgs::PointCloud::ConstPtr &msg){
  // auto t1 = std::chrono::high_resolution_clock::now(); 
  // auto t2 = std::chrono::high_resolution_clock::now(); 
  // printf("CB t: %.6f\n", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1e-6);

  // size_t id = data.id[i] + currid;
  size_t currid = 1024;

  for (size_t i = 0; i < msg->points.size(); i++) {
    // get tracked feature result: [id, (u,v)]
    auto id = static_cast<unsigned int>(msg->channels[0].values[i]);
    auto u = static_cast<float>(msg->points[i].x);
    auto v = static_cast<float>(msg->points[i].y);

    // save to database
    auto data_id = id + currid + 1;
    if(data_id==1984) {
      printf(" [%.9f,(%.3f,%.3f)]\n", msg->header.stamp.toSec(), u, v);
    }
  }
}
