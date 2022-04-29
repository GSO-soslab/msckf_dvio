// ros
#include <ros/ros.h>
// ros msg type header
#include <nortek_dvl/ButtomTrack.h>
#include <nortek_dvl/CurrentProfile.h>

#include "types/msgs.h"

class RemapDVL
{
public:
    RemapDVL() {
        sub_bt = nh.subscribe<nortek_dvl::ButtomTrack> ("/rov/sensors/dvl/buttom_track", 20, &RemapDVL::btCallback, this);
        sub_cp = nh.subscribe<nortek_dvl::CurrentProfile> ("/rov/sensors/dvl/current_profile", 20, &RemapDVL::cpCallback, this);

        pub_bt = nh.advertise<nortek_dvl::ButtomTrack>("/rov/sensors/dvl/buttom_track_remap", 20);
        pub_cp = nh.advertise<nortek_dvl::CurrentProfile>("/rov/sensors/dvl/current_profile_remap", 20);
    }

    ~RemapDVL() {}

    void btCallback(const nortek_dvl::ButtomTrack::ConstPtr& input);

    void cpCallback(const nortek_dvl::CurrentProfile::ConstPtr& input);

    bool mapDvlTime(const msckf_dvio::DvlMsg &in);

    bool mapDvlTime_(const nortek_dvl::ButtomTrack::ConstPtr& input);

private:
    ros::NodeHandle nh;

    ros::Subscriber sub_bt;
    ros::Subscriber sub_cp;
    ros::Publisher pub_bt;
    ros::Publisher pub_cp;

    std::vector<std::tuple<Eigen::Vector3d, double, double>> remap_queue;
    std::vector<msckf_dvio::DvlMsg> remapped_queue;
    double last_integral = 0.0;
    std::string last_flag = "#";

    // 
    std::vector<std::tuple<nortek_dvl::ButtomTrack, double, double>> remap_queue_;
    std::vector<nortek_dvl::ButtomTrack> remapped_queue_;
    double last_integral_ = 0.0;
    std::string last_flag_ = "#";

    // t_imu = t_dvl - t_offset
    double t_offset = 10.645267;
};


void RemapDVL::btCallback(const nortek_dvl::ButtomTrack::ConstPtr& input) {
  if(mapDvlTime_(input)) {
    for(const auto msg:remapped_queue_) {
      nortek_dvl::ButtomTrack msg_bt;
      msg_bt = msg;
      // apply delay time offset
      double correct_t = msg.header.stamp.toSec() - t_offset;
      msg_bt.header.stamp = ros::Time(correct_t);

      // re-publish
      pub_bt.publish(msg_bt);
      printf("t:%.9f\n",msg_bt.header.stamp.toSec());
    }    
  }

  // msckf_dvio::DvlMsg message;
  // message.time = input->header.stamp.toSec();
  // message.v << input->speed.x, input->speed.y, input->speed.z;
  // // remap DVL BT time
  // if(mapDvlTime(message)){
  //   for(const auto msg:remapped_queue) {
  //     nortek_dvl::ButtomTrack msg_bt;
  //     msg_bt.header.frame_id = input->header.frame_id;
  //     msg_bt.header.stamp = ros::Time(msg.time);
  //     msg_bt.speed.x = msg.v(0);
  //     msg_bt.speed.y = msg.v(1);
  //     msg_bt.speed.z = msg.v(2);
  //     printf("t:%.9f\n",msg.time);
  //   }
  // }
}

void RemapDVL::cpCallback(const nortek_dvl::CurrentProfile::ConstPtr& input) {
  // copy data
  nortek_dvl::CurrentProfile msg_cp;
  msg_cp = *input;
  // apply delay offset
  double correct_t = msg_cp.header.stamp.toSec() - t_offset;
  msg_cp.header.stamp = ros::Time(correct_t);
  // re-publish
  pub_cp.publish(msg_cp);
}

bool RemapDVL::mapDvlTime_(const nortek_dvl::ButtomTrack::ConstPtr& input) {
  /***** re-map DVL timestmaps because of decoding larger current profile data in Serial driver *****/
  /***** BT: 0.0, 0.25, 0.5, 0.75, from DVL system time *****/
  bool flag = false;

  //// get integral and fractional part of time
  double integral, fractional; 
  fractional = std::modf(input->header.stamp.toSec(), &integral);

  //// received data in 1 second, now remap
  if(integral != last_integral_ && last_integral_ !=0.0) {
    //// clear last remapped data
    remapped_queue_.clear();
    
    nortek_dvl::ButtomTrack msg_bt;
    switch(remap_queue_.size()) {
      //// take last 4 
      case 7: {
        msg_bt = std::get<0>(remap_queue_.at(3));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(3)) + 0.00);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(4));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(4)) + 0.25);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(5));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(5)) + 0.50);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(6));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(6)) + 0.75);
        remapped_queue_.emplace_back(msg_bt);

        last_flag = "#7";
        break;
      }

      case 6: {
        if(last_flag == "#2") {
          msg_bt = std::get<0>(remap_queue_.at(0));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) - 0.5);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(1));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) - 0.25);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(2));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.0);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(3));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(3)) + 0.25);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(4));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(4)) + 0.5);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(5));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(5)) + 0.75);
          remapped_queue_.emplace_back(msg_bt);
        }
        else {
          //// take last 4
          msg_bt = std::get<0>(remap_queue_.at(2));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.0);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(3));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(3)) + 0.25);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(4));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(4)) + 0.5);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(5));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(5)) + 0.75);
          remapped_queue_.emplace_back(msg_bt);
        }

        last_flag = "#6";
        break;        
      }

      //// take last 4
      case 5: {
        msg_bt = std::get<0>(remap_queue_.at(1));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) + 0.00);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(2));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.25);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(3));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(3)) + 0.5);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(4));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(4)) + 0.75);
        remapped_queue_.emplace_back(msg_bt);

        last_flag = "#5";
        break;
      }

      //// 1 second has 4 bottom track velocity, v1,v2,v3,v4, assign with 0.25,0.5,0.75,1.0 as fractional part
      //// some case: v4_, v1,v2,v3, we also treat this above just keep code easier
      case 4: {
        msg_bt = std::get<0>(remap_queue_.at(0));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) + 0.00);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(1));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) + 0.25);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(2));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.5);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(3));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(3)) + 0.75);
        remapped_queue_.emplace_back(msg_bt);

        last_flag = "#4";
        break;
      }

      //// 1 second has 3 bottom track velocity, lost one measurement during serial transmission
      case 3: {
        //// get fractional
        double v1 = std::get<2>(remap_queue_.at(0));
        double v2 = std::get<2>(remap_queue_.at(1));
        double v3 = std::get<2>(remap_queue_.at(2));
        double avg_1 = (v1+v2)*0.5;
        double avg_2 = (v2+v3)*0.5;

        //// missing one from first set (v1)?, (v2,v3)
        if(avg_1<0.75 && avg_2>0.75) {
          msg_bt = std::get<0>(remap_queue_.at(0));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) + 0.25);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(1));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) + 0.5);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(2));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.75);
          remapped_queue_.emplace_back(msg_bt);
        }
        //// missing one from second set (v1,v2), (v3)?
        else if(avg_1<0.5 && avg_2>0.5) {
          msg_bt = std::get<0>(remap_queue_.at(0));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) + 0.00);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(1));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) + 0.25);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(2));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.5);
          remapped_queue_.emplace_back(msg_bt);
        }
        else {
          msg_bt = std::get<0>(remap_queue_.at(0));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) + 0.00);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(1));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) + 0.25);
          remapped_queue_.emplace_back(msg_bt);

          msg_bt = std::get<0>(remap_queue_.at(2));
          msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(2)) + 0.5);
          remapped_queue_.emplace_back(msg_bt);
        }
        
        last_flag = "#3";
        break;
      }

      case 2: {
        // get fractional
        double v1 = std::get<2>(remap_queue_.at(0));
        double v2 = std::get<2>(remap_queue_.at(1));
        double avg = (v1+v2)*0.5;

        msg_bt = std::get<0>(remap_queue_.at(0));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) + 0.00);
        remapped_queue_.emplace_back(msg_bt);

        msg_bt = std::get<0>(remap_queue_.at(1));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(1)) + 0.25);
        remapped_queue_.emplace_back(msg_bt);

        last_flag = "#2";
        break;
      }

      case 1: {
        msg_bt = std::get<0>(remap_queue_.at(0));
        msg_bt.header.stamp = ros::Time(std::get<1>(remap_queue_.at(0)) + 0.00);
        remapped_queue_.emplace_back(msg_bt);

        last_flag = "#1";
        break;
      }

      //// some other cases
      default : {
        last_flag = "#";
        break;
      }

    }

    remap_queue_.clear();
    flag = true;
  }

  // append new data
  remap_queue_.emplace_back(*input, integral, fractional);
  last_integral_ = integral;

  return flag;
}

bool RemapDVL::mapDvlTime(const msckf_dvio::DvlMsg &in) {
  /***** re-map DVL timestmaps because of decoding larger current profile data in Serial driver *****/
  /***** BT: 0.0, 0.25, 0.5, 0.75, from DVL system set up *****/
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

  // append new data
  remap_queue.emplace_back(in.v, integral, fractional);
  last_integral = integral;

  // // TEST
  // printf("t:%f\n", in.time);

  return flag;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Simple_Example_Node");

  RemapDVL  example;
  ros::spin();

  return 0;
}