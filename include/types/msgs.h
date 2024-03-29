#ifndef MSCKF_TYPE_MSG_H_
#define MSCKF_TYPE_MSG_H_

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

namespace msckf_dvio {
  
/******* ROS message data ********/

struct ImuMsg {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // IMU timestamp
    double time;

    // IMU 3-axis acceleration  
    Eigen::Vector3d a;

    // IMU 3-axis gyro 
    Eigen::Vector3d w;

    // Sort IMU measurement with time
    bool operator<(const ImuMsg &other) const { return time < other.time; }
};

struct DvlMsg {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // timestamp
    double time;

    // 3-axis velocity  
    Eigen::Vector3d v;

    // 3-axis acceleration  
    Eigen::Vector3d a;

    // Sort sensor measurement with time
    bool operator<(const DvlMsg &other) const { return time < other.time; }

    DvlMsg() {}
    DvlMsg(const double &time_, const Eigen::Vector3d &v_, const Eigen::Vector3d &a_)
      : time(time_), v(v_), a(a_) {}
};

struct ImageMsg {

  // Image timestamp
  double time;

  // Image
  cv::Mat image;

  // Sort image with time
  bool operator<(const ImageMsg &other) const { return time < other.time; }
};

struct PressureMsg {

  //  timestamp
  double time;

  // pressure
  double p;

  // Sort data with time
  bool operator<(const PressureMsg &other) const { return time < other.time; }
};


struct FeatureMsg {

  //  timestamp
  double time;

  // u measurement of a image
  std::vector<float> u;

  // v measurement of a image
  std::vector<float> v;

  // id of this feature measurement
  std::vector<unsigned int> id;

  // Sort data with time
  bool operator<(const FeatureMsg &other) const { return time < other.time; }
};

} // namespace msckf_dvio

#endif  //MSCKF_TYPE_MSG_H_