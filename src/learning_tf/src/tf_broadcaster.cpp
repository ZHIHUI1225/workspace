#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <robot_msg/robot_pose.h>
#include <robot_msg/robot_pose_array.h>
#include <robot_msg/target_pose.h>
#include <robot_msg/target_pose_array.h>
#include <string>
#include <iostream>
#include <sstream>
#include <sensor_msgs/Image.h>
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.h"
#include <cmath>
#include <stdlib.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
void poseCallback(const robot_msg::robot_pose_array& msg){
  static tf::TransformBroadcaster br,br34;
  tf::Transform transform, transform34;
  bool flag1=false;
  bool flag2=false; 
  bool flag3=false;
  bool flag4=false;
  geometry_msgs::Point32 robot1;
  geometry_msgs::Point robot2;
  geometry_msgs::Point32 robot3;
  geometry_msgs::Point robot4;
    for (int i = 0; i < msg.robot_pose_array.size(); i++) 
    { int ID=msg.robot_pose_array[i].ID.data;
      
      if (ID == 11)
      {
      
        robot1.x=msg.robot_pose_array[i].position.x;
        robot1.y=msg.robot_pose_array[i].position.y;
        robot1.z=msg.robot_pose_array[i].position.z;
        flag1=true;
      }
      if (ID == 12)
      {
       
        robot2.x=msg.robot_pose_array[i].position.x;
        robot2.y=msg.robot_pose_array[i].position.y;
        flag2=true;
      }
       if (ID == 13)
      {
      
        robot3.x=msg.robot_pose_array[i].position.x;
        robot3.y=msg.robot_pose_array[i].position.y;
        robot3.z=msg.robot_pose_array[i].position.z;
        flag3=true;
      }
      if (ID == 14)
      {
       
        robot4.x=msg.robot_pose_array[i].position.x;
        robot4.y=msg.robot_pose_array[i].position.y;
        flag4=true;
      }
    }
    if (flag1 && flag2)
    {
       transform.setOrigin( tf::Vector3(robot1.x, robot1.y, robot1.z) );
      tf::Quaternion q;
      double yaw=atan2(robot2.y-robot1.y,robot2.x-robot1.x);
      q.setRPY(M_PI, 0,yaw);
      // q.setEuler(0,M_PI,yaw);
  
      
      transform.setRotation(q);
      //   std::stringstream ss;
      // ss << msg.robot_pose_array[i].ID;
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","local"));
    }
     
    if (flag3 && flag4)
    {
       transform34.setOrigin( tf::Vector3(robot3.x, robot3.y, robot3.z) );
      tf::Quaternion q34;
      double yaw34=atan2(robot4.y-robot3.y,robot4.x-robot3.x);
      q34.setRPY(M_PI, 0,yaw34);
      // q.setEuler(0,M_PI,yaw);
      transform34.setRotation(q34);
      //   std::stringstream ss;
      // ss << msg.robot_pose_array[i].ID;
      br34.sendTransform(tf::StampedTransform(transform34, ros::Time::now(), "world","local34"));
    }

}
void imageCallBack(const sensor_msgs::Image::ConstPtr& msg)
	{
cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat frame = cv_ptr->image(cv::Range::all(), cv::Range::all());
    int col=frame.cols;
    int row=frame.rows;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, row,0));
    tf::Quaternion q;
    q.setRPY(M_PI,0,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","map"));
  }

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/robot", 10, &poseCallback);
  ros::Subscriber subimage= node.subscribe("camera/image",10, &imageCallBack);
  ros::spin();
  return 0;
};