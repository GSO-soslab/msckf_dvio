#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sstream>
#include <Eigen/Eigen>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  /*** some test ***/
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(3.13184,0.00322321,3.13023 );  // Create this quaternion from roll/pitch/yaw (in radians)
  myQuaternion.normalize();
  tf2::Matrix3x3 mat(myQuaternion);
  // std::cout<<"Q: "<< myQuaternion[0]<<std::endl;
  // std::cout<<"Mat: "
  //          << "\n " << mat[0][0]<<" "<< mat[0][1] << " " << mat[0][2]
  //          << "\n " << mat[1][0]<<" "<< mat[1][1] << " " << mat[1][2]
  //          << "\n " << mat[2][0]<<" "<< mat[2][1] << " " << mat[2][2]
  //          << std::endl;

  Eigen::Matrix3d R,R_inverse;
  R = Eigen::AngleAxisd(3.13023, Eigen::Vector3d::UnitZ()) * 
            Eigen::AngleAxisd(0.00322321, Eigen::Vector3d::UnitY()) * 
            Eigen::AngleAxisd(3.13184, Eigen::Vector3d::UnitX());
  R_inverse = R.transpose();
  std::cout<<"eigen Mat: \n"<< R_inverse<<std::endl;

  Eigen::Vector3d p, p_new;
  p<<  0.43382845, -0.088165092, -0.30145;

  p_new = -1*R_inverse*p;
  std::cout<<"p: \n"<< p_new<<std::endl;


  /**** Publisher example ****/
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}