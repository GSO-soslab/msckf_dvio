// ros
#include <ros/ros.h>
// sub
#include <nortek_dvl/ButtomTrack.h>    // GLRC Nortek BT msg
#include <nortek_dvl/CurrentProfile.h> // GLRC Nortek CP msg
// pub
#include <geometry_msgs/TwistWithCovarianceStamped.h> // velocity measurement
#include <sensor_msgs/FluidPressure.h>  // pressure measurement

class DvlHelper
{
public:
    DvlHelper(const ros::NodeHandle &nh,
              const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
    {
        sub_glrc_bt = nh_.subscribe<nortek_dvl::ButtomTrack> ("/glrc/bt", 10, &DvlHelper::glrcBtCallback, this);
        sub_glrc_cp = nh_.subscribe<nortek_dvl::CurrentProfile> ("/glrc/cp", 2, &DvlHelper::glrcCpCallback, this);
        
        pub_glrc_velocity = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/glrc/velocity",10);
        pub_glrc_pressure = nh_.advertise<sensor_msgs::FluidPressure>("/glrc/pressure",10);

        nh_private_.param<int>("config/param", param_, 115200);
    }

    ~DvlHelper() {}

    void glrcBtCallback(const nortek_dvl::ButtomTrack::ConstPtr& msg);

    void glrcCpCallback(const nortek_dvl::CurrentProfile::ConstPtr& msg);

    void refresh();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_glrc_bt;
    ros::Subscriber sub_glrc_cp;

    ros::Publisher pub_glrc_velocity;
    ros::Publisher pub_glrc_pressure;

    int param_;
};

void DvlHelper::glrcBtCallback(const nortek_dvl::ButtomTrack::ConstPtr& msg) {

    geometry_msgs::TwistWithCovarianceStamped msg_out;
    msg_out.header = msg->header;
    msg_out.twist.twist.linear.x = msg->speed.x;
    msg_out.twist.twist.linear.y = msg->speed.y;
    msg_out.twist.twist.linear.z = msg->speed.z;
    pub_glrc_velocity.publish(msg_out);
        
    //! TODO: convert figure_of_merit into covariance ??
}

void DvlHelper::glrcCpCallback(const nortek_dvl::CurrentProfile::ConstPtr& msg) {

    sensor_msgs::FluidPressure msg_out;
    msg_out.header = msg->header;
    msg_out.fluid_pressure = msg->pressure; //dBar
    msg_out.variance = pow(msg->pressure_std_dev,2);
    pub_glrc_pressure.publish(msg_out);
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