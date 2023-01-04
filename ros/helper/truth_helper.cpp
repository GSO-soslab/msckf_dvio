
// ros
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class TruthPub
{
public:
    TruthPub()
    {
        position_sub_ = nh.subscribe<geometry_msgs::PointStamped> ("/leica/position", 10, &TruthPub::positionCallback, this);

        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/ground_truth",10);
        path_pub_ = nh.advertise<nav_msgs::Path>("/ground_truth_path",10);
    }

    ~TruthPub()
    {
    }


    void positionCallback(const geometry_msgs::PointStampedConstPtr& msg);

private:
    ros::NodeHandle nh;

    ros::Subscriber position_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;

    nav_msgs::Path path_truth_;

};

void TruthPub::positionCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
    // ground truth path
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "odom";
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z;

    path_truth_.header.frame_id = "odom";
    path_truth_.header.stamp = msg->header.stamp;
    path_truth_.poses.push_back(pose);

    path_pub_.publish(path_truth_);

    // ground truth odom
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = msg->point.x;
    odom.pose.pose.position.y = msg->point.y;
    odom.pose.pose.position.z = msg->point.z;

    odom_pub_.publish(odom);
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "Simple_Example_Node");

     TruthPub  example;

     ros::spin();
   

     return 0;
}