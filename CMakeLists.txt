cmake_minimum_required(VERSION 3.0.2)
project(msckf_dvio)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## CMAKE module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

### Find Library

SET(OpenCV_DIR /home/lin/develop/3rd/opencv/install/opencv-4.2.0/lib/cmake/opencv4/)
find_package(OpenCV 4.2.0 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


# find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(magic_enum CONFIG REQUIRED)


# set(Open3D_DIR /home/lin/develop/3rd/Open3D/install/lib/cmake/Open3D)
# find_package(Open3D)
# if(Open3D_FOUND)  
#   message(STATUS "Found Open3D ${Open3D_VERSION}")
# else()
#   message(STATUS "Open3D Not Found")
# endif()

### Find ROS package
SET(cv_bridge_DIR /home/lin/develop/ros/ros_ws/devel/share/cv_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs rosbag sensor_msgs nav_msgs
  geometry_msgs cv_bridge message_generation tf2
  tf_conversions eigen_conversions std_srvs
  dynamic_reconfigure message_filters image_transport 
  ds_sensor_msgs nortek_dvl tf2_ros tf2_geometry_msgs
  pcl_conversions
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
  # ${CERES_INCLUDE_DIRS}
  # /home/lin/develop/3rd/kalman-cpp
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

add_executable(dvl_helper_node ros/helper/dvl_helper.cpp)
target_link_libraries(dvl_helper_node ${thirdparty_libraries})

add_executable(truth_helper_node ros/helper/truth_helper.cpp)
target_link_libraries(truth_helper_node ${thirdparty_libraries})

##########################################
############### build test ###############
##########################################

# add_executable(test_imu_init_manual test/cpp/imu_init_manual.cpp)
# target_link_libraries(test_imu_init_manual dvio_lib ${thirdparty_libraries})

# add_executable(test_imu_init_rosbag test/cpp/imu_init_rosbag.cpp)
# target_link_libraries(test_imu_init_rosbag dvio_lib ${thirdparty_libraries})

# add_executable(test_ros_rpy test/cpp/ros_rpy.cpp)
# target_link_libraries(test_ros_rpy ${thirdparty_libraries})

# add_executable(test_shared_ptr test/cpp/test_shared_ptr.cpp)
# target_link_libraries(test_shared_ptr ${thirdparty_libraries})

# add_executable(test_set_map test/cpp/test_set_map.cpp)
# target_link_libraries(test_set_map ${thirdparty_libraries})


# add_executable(test_remapDVL test/cpp/remap_dvl.cpp)
# target_link_libraries(test_remapDVL ${thirdparty_libraries})

# add_executable(test_save test/tools_save_map.cpp)
# target_link_libraries(test_save
#   ${catkin_LIBRARIES}
#   Open3D::Open3D
# )
# add_dependencies(test_save ${PROJECT_NAME}_gencfg)

add_executable(test_save test/cpp/save_data.cpp)
target_link_libraries(test_save ${thirdparty_libraries})

add_executable(test_feature test/cpp/feature_database.cpp)
target_link_libraries(test_feature ${thirdparty_libraries})

add_executable(test_cpp test/cpp/c_plus_plus.cpp)
target_link_libraries(test_cpp ${thirdparty_libraries})

add_executable(test_timecost test/cpp/test_timecost.cpp)
target_link_libraries(test_timecost ${thirdparty_libraries})

# add_executable(test_video_stream test/cpp/view_video_stream.cpp)
# target_link_libraries(test_video_stream ${thirdparty_libraries})

# add_executable(test_evaluation test/cpp/evaluate_image_stitch.cpp)
# target_link_libraries(test_evaluation ${thirdparty_libraries})