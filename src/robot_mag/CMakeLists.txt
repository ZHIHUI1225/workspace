cmake_minimum_required(VERSION 3.0.2)
project(robot_msg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
            COMPONENTS 
            roscpp 
            rospy 
            message_generation 
            std_msgs 
            geometry_msgs
)

add_message_files(
  FILES
	robot_pose.msg
	robot_pose_array.msg
	target_pose.msg
	target_pose_array.msg
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)
