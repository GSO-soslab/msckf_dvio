
// ros
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class TruthPub
{
public:
    TruthPub(const ros::NodeHandle &nh,
             const ros::NodeHandle &nh_private) :
      nh_(nh), nh_private_(nh_private)
    {
        nh_private_.param<std::string>("frame_id", frame_id,    "odom");
        nh_private_.param<std::string>("truth_topic_point", truth_topic_point,    "/groundtruth");
        nh_private_.param<std::string>("truth_topic_pose", truth_topic_pose,    "/groundtruth");

        point_sub_ = nh_.subscribe<geometry_msgs::PointStamped> (truth_topic_point, 10, &TruthPub::pointCallback, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped> (truth_topic_pose, 10, &TruthPub::poseCallback, this);

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ground_truth",10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/ground_truth_path",10);
    }

    ~TruthPub()
    {
    }


    void pointCallback(const geometry_msgs::PointStampedConstPtr& msg);

    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;

    nav_msgs::Path path_truth_;

    std::string frame_id;
    std::string truth_topic_point;
    std::string truth_topic_pose;

};

void TruthPub::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    // ground truth path
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = msg->pose.position.x;
    pose.pose.position.y = msg->pose.position.y;
    pose.pose.position.z = msg->pose.position.z;

    path_truth_.header.frame_id = frame_id;
    path_truth_.header.stamp = msg->header.stamp;
    path_truth_.poses.push_back(pose);

    path_pub_.publish(path_truth_);
}

void TruthPub::pointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
    // ground truth path
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z;

    path_truth_.header.frame_id = frame_id;
    path_truth_.header.stamp = msg->header.stamp;
    path_truth_.poses.push_back(pose);

    path_pub_.publish(path_truth_);

    // ground truth odom
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = frame_id;
    odom.pose.pose.position.x = msg->point.x;
    odom.pose.pose.position.y = msg->point.y;
    odom.pose.pose.position.z = msg->point.z;

    odom_pub_.publish(odom);
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "Simple_Example_Node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

     TruthPub  example(nh, nh_private);

     ros::spin();
   

     return 0;
}