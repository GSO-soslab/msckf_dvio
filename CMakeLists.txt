cmake_minimum_required(VERSION 3.0.2)
project(msckf_dvio)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

### Find Library

# SET(OpenCV_DIR /home/lin/develop/3rd/opencv/install/opencv-4.2.0/lib/cmake/opencv4/)
# find_package(OpenCV 4.2.0 REQUIRED)
# message(STATUS "OpenCV library status:")
# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)

# set(Open3D_DIR /home/lin/develop/3rd/Open3D/install/lib/cmake/Open3D)
# find_package(Open3D)
# if(Open3D_FOUND)  
#   message(STATUS "Found Open3D ${Open3D_VERSION}")
# else()
#   message(STATUS "Open3D Not Found")
# endif()

### Find ROS package
# SET(cv_bridge_DIR /home/lin/develop/ros/ros_ws/devel/share/cv_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs rosbag sensor_msgs nav_msgs
  geometry_msgs cv_bridge message_generation
  nortek_dvl tf_conversions eigen_conversions
  dynamic_reconfigure message_filters image_transport tf2
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES feature_tracker
 CATKIN_DEPENDS roscpp rospy std_msgs rosbag sensor_msgs cv_bridge dynamic_reconfigure
                geometry_msgs message_runtime nortek_dvl tf_conversions eigen_conversions
                message_filters image_transport visualization_msgs tf nav_msgs tf2
#  DEPENDS system_lib
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
  # ${OpenCV_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  # /home/lin/develop/3rd/kalman-cpp
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  # ${OpenCV_LIBRARIES}
)

##########################################
############### build lib ###############
##########################################
add_library(dvio_lib SHARED
  include/core/msckf_manager.cpp
  include/core/imu_initializer.cpp
  include/core/state.cpp
)
target_link_libraries(dvio_lib ${thirdparty_libraries})


##########################################
############### build src ################
##########################################
add_executable(dvio_node src/ros_node.cpp)
target_link_libraries(dvio_node dvio_lib ${thirdparty_libraries})


##########################################
############### build test ###############
##########################################
add_executable(test_imu_init_manual test/cpp/imu_init_manual.cpp)
target_link_libraries(test_imu_init_manual dvio_lib ${thirdparty_libraries})

add_executable(test_imu_init_rosbag test/cpp/imu_init_rosbag.cpp)
target_link_libraries(test_imu_init_rosbag dvio_lib ${thirdparty_libraries})


add_executable(test_ros_rpy test/cpp/ros_rpy.cpp)
target_link_libraries(test_ros_rpy ${thirdparty_libraries})


add_executable(test_eigen test/cpp/eigen.cpp)
target_link_libraries(test_eigen ${thirdparty_libraries})

add_executable(test_remapDVL test/cpp/remap_dvl.cpp)
target_link_libraries(test_remapDVL ${thirdparty_libraries})

# add_executable(test_save test/tools_save_map.cpp)
# target_link_libraries(test_save
#   ${catkin_LIBRARIES}
#   Open3D::Open3D
# )
# add_dependencies(test_save ${PROJECT_NAME}_gencfg)

