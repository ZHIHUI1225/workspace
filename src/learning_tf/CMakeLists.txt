cmake_minimum_required(VERSION 3.0.2)
project(learning_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)

find_package(
catkin REQUIRED COMPONENTS
message_generation
nav_msgs
roscpp
rospy
sensor_msgs
std_msgs
geographic_msgs
geometry_msgs
std_srvs
tf2
tf
tf2_ros
tf2_sensor_msgs
tf2_geometry_msgs
image_transport
OpenCV REQUIRED
cv_bridge
robot_msg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCv_INCLUDE_DIRS})


add_executable(tf_broadcaster src/tf_broadcaster.cpp )
target_link_libraries(tf_broadcaster  ${catkin_LIBRARIES} ${OpenCv_LIBRARIES} ${Boost_LIBRARIES})

