cmake_minimum_required(VERSION 3.0.2)
project(msckf_dvio)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## CMAKE module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

### Find Library
# SET(OpenCV_DIR /home/lin/develop/3rd/opencv/install/opencv-4.2.0/lib/cmake/opencv4/)
find_package(OpenCV 4.2.0 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

### Find ROS package
# SET(cv_bridge_DIR /home/lin/develop/ros/ros_ws/devel/share/cv_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs rosbag sensor_msgs nav_msgs
  geometry_msgs cv_bridge message_generation tf2
  tf_conversions eigen_conversions std_srvs
  dynamic_reconfigure message_filters image_transport 
  tf2_ros tf2_geometry_msgs pcl_conversions
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs rosbag sensor_msgs cv_bridge dynamic_reconfigure
                geometry_msgs message_runtime tf_conversions eigen_conversions
                message_filters image_transport visualization_msgs tf nav_msgs tf2 std_srvs
                tf2_ros tf2_geometry_msgs pcl_conversions
 DEPENDS PCL EIGEN3 SUITESPARSE
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  magic_enum::magic_enum
)

##########################################
############### build lib ###############
##########################################
add_library(dvio_lib SHARED

  include/initializer/initializer_dvl_aided.cpp
  include/initializer/initializer_setting.cpp
  
  include/manager/msckf_manager.cpp

  include/msckf/state.cpp
  include/msckf/predictor.cpp
  include/msckf/updater.cpp

  include/feature/Feature.cpp
  include/feature/triangulation.cpp

  include/tracker/TrackBase.cpp
  include/tracker/TrackKLT.cpp
  include/tracker/TrackFeature.cpp
)
target_link_libraries(dvio_lib ${thirdparty_libraries})


##########################################
############### build ros ################
##########################################

add_executable(dvio_node ros/node.cpp ros/visualizer.cpp)
target_link_libraries(dvio_node dvio_lib ${thirdparty_libraries})

##########################################
############### build test ###############
##########################################
