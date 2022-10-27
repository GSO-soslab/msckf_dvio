// ros
#include <ros/ros.h>
// sub
#include <nortek_dvl/ButtomTrack.h>    // GLRC Nortek BT msg
#include <nortek_dvl/CurrentProfile.h> // GLRC Nortek CP msg
#include <ds_sensor_msgs/NortekDF21.h> // Alaska Nortek BT msg
#include <ds_sensor_msgs/NortekDF3.h>  // Alaska Nortek CP msg

// pub
#include <geometry_msgs/TwistWithCovarianceStamped.h> // velocity measurement
#include <sensor_msgs/FluidPressure.h>  // pressure measurement
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>

#define PI 3.1415926


class DvlHelper
{
public:
    DvlHelper(const ros::NodeHandle &nh,
              const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
    {
        sub_glrc_bt = nh_.subscribe<nortek_dvl::ButtomTrack> ("/glrc/bt", 10, &DvlHelper::glrcBtCallback, this);
        sub_glrc_cp = nh_.subscribe<nortek_dvl::CurrentProfile> ("/glrc/cp", 2, &DvlHelper::glrcCpCallback, this);

        sub_alaska_bt = nh_.subscribe<ds_sensor_msgs::NortekDF21> ("/alaska/bt", 10, &DvlHelper::alaskaBtCallback, this);
        sub_alaska_cp = nh_.subscribe<ds_sensor_msgs::NortekDF3> ("/alaska/cp", 2, &DvlHelper::alaskaCpCallback, this);
        
        pub_velocity = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/velocity",10);
        pub_pressure = nh_.advertise<sensor_msgs::FluidPressure>("/pressure",10);
        pub_pointcloud    = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud", 5);


        nh_private_.param<double>("sound_speed", sound_speed_, 1500);
        nh_private_.param<double>("beam_angle", beam_angle_, 25);
    }

    ~DvlHelper() {}

    void glrcBtCallback(const nortek_dvl::ButtomTrack::ConstPtr& msg);

    void glrcCpCallback(const nortek_dvl::CurrentProfile::ConstPtr& msg);

    void alaskaBtCallback(const ds_sensor_msgs::NortekDF21::ConstPtr& msg);

    void alaskaCpCallback(const ds_sensor_msgs::NortekDF3::ConstPtr& msg);

    void refresh();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_glrc_bt;
    ros::Subscriber sub_glrc_cp;

    ros::Subscriber sub_alaska_bt;
    ros::Subscriber sub_alaska_cp;

    ros::Publisher pub_velocity;
    ros::Publisher pub_pressure;
    ros::Publisher pub_pointcloud;

    double sound_speed_;
    double beam_angle_;
};

void DvlHelper::glrcBtCallback(const nortek_dvl::ButtomTrack::ConstPtr& msg) {

    // 3-axis velocity
    //! TODO: convert figure_of_merit into covariance ??
    geometry_msgs::TwistWithCovarianceStamped msg_out;
    msg_out.header = msg->header;
    msg_out.twist.twist.linear.x = msg->speed.x;
    msg_out.twist.twist.linear.y = msg->speed.y;
    msg_out.twist.twist.linear.z = msg->speed.z;
    pub_velocity.publish(msg_out);
    
    // 4 beams generated pointcloud with only XYZ property
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");    
    modifier.resize(4); 

    // re-generate 3D location of target point
    std::vector<Eigen::Vector3d> points;
    auto distance = msg->vertical_distance;
    double beam_angle = beam_angle_*PI/180;

    double beam_azimuth[] = {PI/4.0, -PI/4.0, -3.0*PI/4.0, 3.0*PI/4.0};
    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d pt;
        pt(0) = distance[i] * tan(beam_angle) * cos(beam_azimuth[i]);
        pt(1) = distance[i] * tan(beam_angle) * sin(beam_azimuth[i]);
        pt(2) = distance[i];
        points.push_back(pt);
    }

    // setup the points XYZ
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(cloud_msg, "z");

    for (size_t i = 0; i < 4; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        const Eigen::Vector3d& point = points.at(i);
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
    }

    // publish
    cloud_msg.header = msg->header;
    pub_pointcloud.publish(cloud_msg);
}

void DvlHelper::glrcCpCallback(const nortek_dvl::CurrentProfile::ConstPtr& msg) {

    sensor_msgs::FluidPressure msg_out;
    msg_out.header = msg->header;
    msg_out.fluid_pressure = msg->pressure; //dBar
    msg_out.variance = pow(msg->pressure_std_dev,2);
    pub_pressure.publish(msg_out);
}

void DvlHelper::alaskaBtCallback(const ds_sensor_msgs::NortekDF21::ConstPtr& msg) {

    //// publish 3-axis velocity
    //! TODO: convert figure_of_merit into covariance ??
    geometry_msgs::TwistWithCovarianceStamped msg_out;

    msg_out.header = msg->header;
    msg_out.twist.twist.linear.x = msg->velX;
    msg_out.twist.twist.linear.y = msg->velY;
    if (msg->velZ1 == -32.768f && msg->velZ2 != -32.768f) 
        msg_out.twist.twist.linear.z = msg->velZ2;
    else if (msg->velZ1 != -32.768f && msg->velZ2 == -32.768f)
        msg_out.twist.twist.linear.z = msg->velZ1;
    else 
        msg_out.twist.twist.linear.z = (msg->velZ1 + msg->velZ2)/ 2.0;

    pub_velocity.publish(msg_out);
    
    //// publish pointcloud message
    // 4 beams generated pointcloud with only XYZ property
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");    
    modifier.resize(4); 

    // re-generate 3D location of target point
    std::vector<Eigen::Vector3d> points;
    double beam_angle = beam_angle_*PI/180;

    double beam_azimuth[] = {PI/4.0, -PI/4.0, -3.0*PI/4.0, 3.0*PI/4.0};
    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d pt;
        pt(0) = msg->distBeam[i] * tan(beam_angle) * cos(beam_azimuth[i]);
        pt(1) = msg->distBeam[i] * tan(beam_angle) * sin(beam_azimuth[i]);
        pt(2) = msg->distBeam[i];
        points.push_back(pt);
    }

    // setup the points XYZ
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(cloud_msg, "z");

    for (size_t i = 0; i < 4; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        const Eigen::Vector3d& point = points.at(i);
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
    }

    // publish
    cloud_msg.header = msg->header;
    pub_pointcloud.publish(cloud_msg);

    //// publish pressure message
    sensor_msgs::FluidPressure msg_pressure;
    msg_pressure.header = msg->header;
    //Bar-> meter
    msg_pressure.fluid_pressure = msg->pressure * 10; 
    msg_pressure.variance = 0;
    pub_pressure.publish(msg_pressure);
}

void DvlHelper::alaskaCpCallback(const ds_sensor_msgs::NortekDF3::ConstPtr& msg) {

    sensor_msgs::FluidPressure msg_out;
    msg_out.header = msg->header;
    //Bar
    msg_out.fluid_pressure = msg->pressure * 10; 
    msg_out.variance = 0;
    pub_pressure.publish(msg_out);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "DVL_Helper");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  DvlHelper  node(nh, nh_private);

  ros::spin();

//   ros::Rate loop_rate(10);
//   while (ros::ok())
//   {
//     node.refresh();
//     ros::spinOnce();
//     loop_rate.sleep();
//   }

  return 0;
}