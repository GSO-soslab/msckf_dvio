#include <mutex>
#include <vector>
// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
// ros msg type header
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZ PointType;

class TestDataNode
{
public:
    TestDataNode(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private) 
    : nh_(nh), nh_private_(nh_private)
    {

        // sync stereo images
        sync_left.subscribe(nh_, "/image_left", 20);
        sync_right.subscribe(nh_, "/image_right", 20);
        sync_.reset(new Sync(MySyncPolicy(10), sync_left, sync_right));
        sync_->registerCallback(boost::bind(&TestDataNode::stereoCallback, this, _1, _2));

        // sub individual sensors
        sub_img = nh_.subscribe("/image", 1, &TestDataNode::imgCallback, this);
        sub_cloud = nh_.subscribe("/cloud", 1, &TestDataNode::cloudCallback, this);
        sub_odom = nh_.subscribe("/odom", 1, &TestDataNode::odomCallback, this);

        service_multi = nh_.advertiseService("/cmd",&TestDataNode::srvCallback, this);
        service_img = nh_.advertiseService("/save_img",&TestDataNode::srvImgCallback, this);
        
        // pub = nh.advertise<sensor_msgs::Image>("/test/sonar_image",1);

        // parameters
        nh_private_.param<std::string>("img_path", img_path, "/home/lin/Desktop/");
        nh_private_.param<double>("save_duration", save_duration, -1.0);
        
    }

    ~TestDataNode() {}

    void stereoCallback(const sensor_msgs::Image::ConstPtr& msg_left, const sensor_msgs::Image::ConstPtr& msg_right);

    void imgCallback(const sensor_msgs::Image::ConstPtr& msg);

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    bool srvCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

    bool srvImgCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

    void refresh();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // publish error
    message_filters::Subscriber<sensor_msgs::Image> sync_left;
    message_filters::Subscriber<sensor_msgs::Image> sync_right;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    ros::Subscriber sub_img;
    ros::Subscriber sub_cloud;
    ros::Subscriber sub_odom;

    ros::ServiceServer service_multi;
    ros::ServiceServer service_img;

    ros::Publisher pub;

    std::mutex buffer_mutex;
    std::vector<sensor_msgs::Image::ConstPtr> buffer_image;
    std::vector<sensor_msgs::PointCloud2::ConstPtr> buffer_cloud;
    std::vector<nav_msgs::Odometry::ConstPtr> buffer_odom;

    // parameters
    std::string img_path;
    double save_duration;

    // save for multi-sensor purpose
    std::atomic<bool> save{false};
    int count = 0;

    // save for individual image
    std::atomic<bool> save_img{false};
    double last_img_t = 0;
    int count_img = 0;
};


bool TestDataNode::srvImgCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {
  res.success = true;
  res.message = "received";

  if(!save_img){
    save_img = true;
    printf("\nStart saving individual image now!\n");
  }
  else{
    save_img = false;
    printf("\nStop saving individual image now!\n");
  }

  return true;
}

bool TestDataNode::srvCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {
  res.success = true;
  res.message = "received";

  save = true;

  return true;
}


void TestDataNode::stereoCallback(const sensor_msgs::Image::ConstPtr& msg_left, 
                                  const sensor_msgs::Image::ConstPtr& msg_right) {
  // save data based on given time duration 
  if(save_duration!=-1 && save_duration >0 && save_img){
    if (msg_right->header.stamp.toSec() - last_img_t > save_duration ){
      // get timestamp
      last_img_t = msg_right->header.stamp.toSec();
      count_img++;

      // convert ros to opencv format
      cv_bridge::CvImagePtr cv_ptr0;
      try{
          cv_ptr0 = cv_bridge::toCvCopy(msg_left, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e){
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      cv_bridge::CvImagePtr cv_ptr1;
      try{
          cv_ptr1 = cv_bridge::toCvCopy(msg_right, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e){
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      // save image
      std::string saved_path_left = img_path + "left/" + std::to_string(last_img_t)  + ".jpg";
      std::string saved_path_right = img_path + "right/" + std::to_string(last_img_t)  + ".jpg";

      bool check0 = cv::imwrite(saved_path_left, cv_ptr0->image);
      bool check1 = cv::imwrite(saved_path_right, cv_ptr1->image);

      if(check0 && check1){
        printf("Saved: t:%lf,count:%d\n", last_img_t, count_img);
      }
      else{
        printf("save failed!\n");
      }
    }
  }         

  // save all data
  if(save_duration ==-1 && save_img){
    // get timestamp
    last_img_t = msg_right->header.stamp.toSec();
    count_img++;

    // convert ros to opencv format
    cv_bridge::CvImagePtr cv_ptr0;
    try{
        cv_ptr0 = cv_bridge::toCvCopy(msg_left, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImagePtr cv_ptr1;
    try{
        cv_ptr1 = cv_bridge::toCvCopy(msg_right, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // save image
    std::string saved_path_left = img_path + "left/" + std::to_string(last_img_t)  + ".jpg";
    std::string saved_path_right = img_path + "right/" + std::to_string(last_img_t)  + ".jpg";

    bool check0 = cv::imwrite(saved_path_left, cv_ptr0->image);
    bool check1 = cv::imwrite(saved_path_right, cv_ptr1->image);

    if(check0 && check1){
      printf("Saved stereo: t:%lf,count:%d\n", last_img_t, count_img);
    }
    else{
      printf("save failed!\n");
    }
  }  

}

void TestDataNode::imgCallback(const sensor_msgs::Image::ConstPtr& msg) {
//     buffer_mutex.lock();

//     buffer_image.emplace_back(msg);

//     // save data based on given time duration 
//     if(save_duration!=-1 && save_duration >0 && save_img){
//       if (msg->header.stamp.toSec() - last_img_t > save_duration ){
//         // set global values
//         last_img_t = msg->header.stamp.toSec();
//         count_img++;

//         // convert ros to opencv format
//         cv_bridge::CvImagePtr cv_ptr;
//         try{
//             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         }
//         catch (cv_bridge::Exception& e){
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//             return;
//         }

//         // save image
//         std::string saved_path = img_path + std::to_string(last_img_t)  + ".jpg";
//         bool check = cv::imwrite(saved_path, cv_ptr->image);
//         if(check){
//           printf("Saved: %s,t:%lf,count:%d\n", saved_path.c_str(), last_img_t, count_img);
//         }
//         else{
//           printf("save failed!\n");
//         }
//       }
//     }

//     // save all the data
//     if(save_duration == -1.0 && save_img) {
//       // convert ros to opencv format
//       cv_bridge::CvImagePtr cv_ptr;
//       try{
//           cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//       }
//       catch (cv_bridge::Exception& e){
//           ROS_ERROR("cv_bridge exception: %s", e.what());
//           return;
//       }

//       last_img_t = msg->header.stamp.toSec();
//       count_img++;

//       // save image
//       std::string saved_path = img_path + std::to_string(last_img_t)  + ".jpg";
//       bool check = cv::imwrite(saved_path, cv_ptr->image);
//       if(check){
//         printf("Saved: %s,t:%lf,count:%d\n", saved_path.c_str(), last_img_t, count_img);
//       }
//       else{
//         printf("save failed!\n");
//       }

//     }

//     // erase for buffer overflow
//     if(buffer_image.size()>100)
//         buffer_image.erase(buffer_image.begin());

//     buffer_mutex.unlock();
}

void TestDataNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  buffer_mutex.lock();

  buffer_cloud.emplace_back(msg);

  // erase for buffer overflow
  if(buffer_cloud.size()>100)
    buffer_cloud.erase(buffer_cloud.begin());

  buffer_mutex.unlock();
}

void TestDataNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  buffer_mutex.lock();
  
  buffer_odom.emplace_back(msg);

  // erase for buffer overflow
  if(buffer_odom.size()>100)
    buffer_odom.erase(buffer_odom.begin());

  buffer_mutex.unlock();
}

void TestDataNode::refresh(){
  if(save) {
    save =false;
    count++;

    double odom_time = -1;

    buffer_mutex.lock();

    //// get odom 
    if(buffer_odom.size()>0){
      odom_time = buffer_odom.back()->header.stamp.toSec();

      //// JPL q_I_G equal to Hamilton q_G_I
      std::cout<<"\n++++++++++++++++ No."<<count<<" +++++++++++++++++\n";
      printf("\nIMU state with t:%f and count:%d \n", odom_time, count);
      std::cout<<"  q_O_I:\n";
      std::cout<<" x: "<< buffer_odom.back()->pose.pose.orientation.x;
      std::cout<<" y: "<< buffer_odom.back()->pose.pose.orientation.y;
      std::cout<<" z: "<< buffer_odom.back()->pose.pose.orientation.z;
      std::cout<<" w: "<< buffer_odom.back()->pose.pose.orientation.w<<std::endl;
      std::cout<<"  p_O_I:\n";
      std::cout<<" x: "<< buffer_odom.back()->pose.pose.position.x;
      std::cout<<" y: "<< buffer_odom.back()->pose.pose.position.y;
      std::cout<<" z: "<< buffer_odom.back()->pose.pose.position.z<<std::endl;
    }
    else{
      printf("no odom found\n");
    }

    //// get dvl pointcloud 
    if(odom_time != -1 && buffer_cloud.size()>0){

      double cloud_time = buffer_cloud.back()->header.stamp.toSec();
      double d_t = cloud_time - odom_time;
      printf("\ncloud time - odom_time: %f\n", d_t);

      if(abs(d_t) < 0.2) {
        // convert ros to pcl format
        pcl::PointCloud<PointType>::Ptr dvl_cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*buffer_cloud.back(), *dvl_cloud);

        // save
        printf("DVL pointcloud with time:%f and count:%d \n", cloud_time, count);
        for( auto it= dvl_cloud->begin(); it!= dvl_cloud->end(); it++)
        {
          std::cout<<"  x: "<<it->x<< " y: "<< it->y << " z: "<< it->z<<std::endl; 
        }
      }

    }
    else
      printf("odom: sth wrong\n");


    //// get image 
    if(odom_time != -1 && buffer_image.size()>0){

      double image_time = buffer_image.back()->header.stamp.toSec();
      double d_t = image_time - odom_time;
      printf("\nimage time - odom_time: %f\n", d_t);

      if(abs(d_t) < 0.2) {

        // convert ros to opencv format
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(*buffer_image.back(), sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //save:
        std::string path = "/home/lin/Desktop/temp/test/";
        bool check = cv::imwrite(path + std::to_string(odom_time)  + ".jpg", cv_ptr->image);
        if(check)
          printf("Saved img at t:%lf, count:%d\n", image_time, count);
        else
          printf("save failed!\n");
      }

    }
    else
      printf("image: sth wrong\n");

    buffer_mutex.unlock();
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "Test_Node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  TestDataNode  node(nh, nh_private);

  ros::Rate loop_rate(15);

  while (ros::ok())
  {
    node.refresh();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}