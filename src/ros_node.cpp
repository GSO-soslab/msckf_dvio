#include "ros_node.h"

#include <thread>

namespace msckf_dvio
{

RosNode::RosNode(ros::NodeHandle nh) :
  nh_(nh)
{
  // get parameters and feed to manager system
  parameters = loadParameters();

  manager = std::make_shared<MsckfManager>(parameters);

  // ROS related
  imu_sub_ = nh_.subscribe("imu", 2000, &RosNode::imuCallback, this);
  dvl_sub_ = nh_.subscribe("dvl", 200, &RosNode::dvlCallback, this);
  image_sub_ = nh_.subscribe("image", 200, &RosNode::imageCallback, this);
}    

Params RosNode::loadParameters() {

  Params params;

/******************** State ********************/
  //===== IMU =====//
  nh_.param<double>("IMU/gravity", params.gravity, 9.81);

  //===== DVL =====//

  //// get DVL extrinsic transformation matrix between IMU and DVl
  XmlRpc::XmlRpcValue ros_param_list;
  nh_.getParam("DVL/T_I_D", ros_param_list);
  //// get DVL timeoffset
  nh_.param<double>("DVL/timeoffset_I_D", params.timeoffset_I_D, 0.0);
  //// get DVL scale factor
  nh_.param<double>("DVL/scale", params.scale, 0.0);

  //// convert matrix into pose (q_I_D, p_I_D)
  Eigen::Matrix4d T_I_D;
  ROS_ASSERT(ros_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < ros_param_list.size(); ++i) {
    for(int32_t j=0; j<ros_param_list[i].size(); ++j) {
      T_I_D(i,j) = static_cast<double>(ros_param_list[i][j]);
    }
  }

  Eigen::Matrix<double, 7, 1> dvl_extrinsics;
  dvl_extrinsics.block(0, 0, 4, 1) = toQuaternion(T_I_D.block(0, 0, 3, 3));
  dvl_extrinsics.block(4, 0, 3, 1) = T_I_D.block(0, 3, 3, 1);
  params.dvl_extrinsics = dvl_extrinsics;

  //===== Camera =====//
  // nh_.param<double>("Camera/timeoffset_I_C", params.timeoffset_I_C, 0.0);

/******************** System ********************/
  nh_.param<int>("backend_hz", params.backend_hz, 20);
  nh_.param<int>("imu_init_mode", params.imu_init_mode, 1);
  nh_.param<int>("imu_windows", params.imu_windows, 20);
  nh_.param<double>("imu_delta_var_1", params.imu_delta_var_1, 0.03);
  nh_.param<double>("imu_delta_var_2", params.imu_delta_var_2, 0.1);
  nh_.param<double>("imu_delta", params.imu_delta, 0.07);
  nh_.param<int>("dvl_windows", params.dvl_windows, 4);
  nh_.param<double>("dvl_delta", params.dvl_delta, 0.05);

  return params;
}

void RosNode::imuCallback(const sensor_msgs::ImuConstPtr &msg) {

  ImuMsg message;
  message.time = msg->header.stamp.toSec();
  message.a << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  message.w << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  manager->feedImu(message);
}

void RosNode::dvlCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg) {
  DvlMsg message;
  message.time = msg->header.stamp.toSec();
  message.v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;

  //// map DVL timestamp because serial driver with 115200 has issue when current profile is set to maximum
  if(mapDvlTime(message))
    manager->feedDvl(remapped_queue);

  // manager->feedDvl(message); 
}


// TODO: check if feature tracking in image callback will effect IMU callback(overflow, bad imu-image align)
void RosNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

  cv_bridge::CvImagePtr cv_ptr;
  try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  ImageMsg message;
  message.image = cv_ptr->image;
  message.time = msg->header.stamp.toSec();;

  manager->feedCamera(message);
}

void RosNode::process() {
  
  int sleep_t = 1.0/parameters.backend_hz *1000.0;


  while(1) {
    // do the ekf stuff
    manager->backend();

    std::chrono::milliseconds dura(sleep_t);
    std::this_thread::sleep_for(dura);
  }
}

bool RosNode::mapDvlTime(const DvlMsg &in) {
  /***** re-map DVL timestmaps because of decoding larger current profile data in Serial driver *****/
  /***** BT(0.25), BT(0.5), CP(0.75), BT(1.0) *****/
  bool flag = false;

  //// get integral and fractional part of time
  double integral, fractional; 
  fractional = std::modf(in.time, &integral);

  //// received data in 1 second, now remap
  if(integral != last_integral && last_integral !=0.0) {
    //// clear last remapped data
    remapped_queue.clear();

    // // TEST
    // file.open(file_path, std::ios_base::app);//std::ios_base::app
    // file << std::setprecision(19);

    switch(remap_queue.size()) {
      //// take last 4 
      case 7: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.00, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.25, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(5)) + 0.50, std::get<0>(remap_queue.at(5)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(6)) + 0.75, std::get<0>(remap_queue.at(6)), Eigen::Vector3d(0,0,0));

        last_flag = "#7";
        break;
      }
      
      case 6: {

        if(last_flag == "#2") {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) - 0.50, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) - 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.00, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.25, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.50, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(5)) + 0.75, std::get<0>(remap_queue.at(5)), Eigen::Vector3d(0,0,0));
        }
        else {
          //// take last 4
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.00, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.25, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.50, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(5)) + 0.75, std::get<0>(remap_queue.at(5)), Eigen::Vector3d(0,0,0));
        }

        last_flag = "#6";
        break;
      }
      
      //// take last 4
      case 5: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.00, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.25, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.50, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(4)) + 0.75, std::get<0>(remap_queue.at(4)), Eigen::Vector3d(0,0,0));

        last_flag = "#5";
        break;
      }

      //// 1 second has 4 bottom track velocity, v1,v2,v3,v4, assign with 0.25,0.5,0.75,1.0 as fractional part
      //// some case: v4_, v1,v2,v3, we also treat this above just keep code easier
      case 4: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.50, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(3)) + 0.75, std::get<0>(remap_queue.at(3)), Eigen::Vector3d(0,0,0));

        last_flag = "#4";
        break;
      }

      //// 1 second has 3 bottom track velocity, lost one measurement during serial transmission
      case 3: {
        //// get fractional
        double v1 = std::get<2>(remap_queue.at(0));
        double v2 = std::get<2>(remap_queue.at(1));
        double v3 = std::get<2>(remap_queue.at(2));
        double avg_1 = (v1+v2)*0.5;
        double avg_2 = (v2+v3)*0.5;

        //// missing one from first set (v1)?, (v2,v3)
        if(avg_1<0.75 && avg_2>0.75) {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.25, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.50, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.75, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        }
        //// missing one from second set (v1,v2), (v3)?
        else if(avg_1<0.5 && avg_2>0.5) {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.50, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        }
        else {
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
          remapped_queue.emplace_back(std::get<1>(remap_queue.at(2)) + 0.50, std::get<0>(remap_queue.at(2)), Eigen::Vector3d(0,0,0));
        }
        
        last_flag = "#3";
        break;
      }

      case 2: {
        // get fractional
        double v1 = std::get<2>(remap_queue.at(0));
        double v2 = std::get<2>(remap_queue.at(1));
        double avg = (v1+v2)*0.5;

        remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(1)) + 0.25, std::get<0>(remap_queue.at(1)), Eigen::Vector3d(0,0,0));
        
        last_flag = "#2";
        break;
      }

      case 1: {
        remapped_queue.emplace_back(std::get<1>(remap_queue.at(0)) + 0.00, std::get<0>(remap_queue.at(0)), Eigen::Vector3d(0,0,0));

        last_flag = "#1";
        break;
      }

      //// some other cases
      default : {
        last_flag = "#";
        break;
      }
    }

    // file.close();

    remap_queue.clear();
    flag = true;
  }

  remap_queue.emplace_back(in.v, integral, fractional);
  last_integral = integral;

  // // TEST
  // printf("t:%f\n", in.time);

  return flag;
}

} // namespace name


int main(int argc, char **argv) {
  ros::init(argc, argv, "MSCKF_DVIO_Node"); 

  ROS_INFO("\n============================================\n"
           "   !!!!!!!!!! MSCKF Node start: !!!!!!!!!! "
           "\n============================================\n");
  
  ros::NodeHandle nh;

  msckf_dvio::RosNode node(nh);

  std::thread backendThread{&msckf_dvio::RosNode::process, &node};

  ros::spin();

  // backendThread.join();

  return 0;
}