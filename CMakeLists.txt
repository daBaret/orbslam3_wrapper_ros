cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_wrapper)

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
    $ENV{HOME}/ros/orbslam3_ws/orb-slam3
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package (catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   cv_bridge
   image_transport
   tf
   sensor_msgs
   message_generation
   geometry_msgs
)

find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3)
MESSAGE("Eigen 3 VERSION:")
MESSAGE(${Eigen3_VERSION})

find_package(Pangolin REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    image_pos.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    sensor_msgs
    geometry_msgs
)

catkin_package (
   CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs message_runtime tf_conversions
   LIBRARIES {PROJECT_NAME}
)

include_directories(
   ${ORB_SLAM3_DIR}
   ${ORB_SLAM3_DIR}/Thirdparty/Sophus
   ${ORB_SLAM3_DIR}/include
   ${ORB_SLAM3_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${catkin_INCLUDE_DIRS}
   ${Pangolin_INCLUDE_DIRS}
)

# Build ROS nodes
set(LIBS
   ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
)


# grid mapping mono sub node
add_executable (${PROJECT_NAME}_mono
   src/ros_mono.cc
   src/wrapper_helper.cc
)
target_link_libraries(${PROJECT_NAME}_mono
   ${LIBS}
)

