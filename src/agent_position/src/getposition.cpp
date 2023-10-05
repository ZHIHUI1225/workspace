//订阅图像话题，并显示图像
#ifndef _GETPOSITION_CPP_
#define _GETPOSITION_CPP_

#include "getposition.hpp"

#include <ros/ros.h>                         //ros.h包含大部分通用的ROS头文件
#include <opencv2/highgui.hpp>               //opencvGUI组件
#include <opencv2/opencv.hpp>               //opencv.hppp包含大部分通用的OpenCV头文件
#include <image_transport/image_transport.h> //image_transport实现图像传输
#include <cv_bridge/cv_bridge.h>             //cv_bridge提供ROS对OpenCV格式的接口功能
using namespace cv;
using namespace std;

Visualize::Visualize()
{

	image_sub = n.subscribe("camera/image",10, &Visualize::imageCallBack,this);
  line_without_QR_pub = n.advertise<sensor_msgs::PointCloud>("/line_without_QR", 1);
  Red_points_pub = n.advertise<sensor_msgs::PointCloud>("/red_points", 1);
  Red_points34_pub = n.advertise<sensor_msgs::PointCloud>("/red_points34", 1);
  //listener_ = new tf::TransformListener(n, ros::Duration(5.0), true);
  robot_pub_ = n.advertise<robot_msg::robot_pose_array>("/robot", 1);
  obstacle_pub_ = n.advertise<robot_msg::robot_pose_array>("/obstacle", 1);
  
}  

Visualize::~Visualize()
{
	cv::destroyAllWindows();
  ROS_INFO("VISULIZE NODE CLOSED");
}

void Visualize::Execute()
{
  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    //point world to map
    // geometry_msgs::Pose input;
    // sensor_msgs::PointCloud pc_map;
    // pc_map.header.frame_id = "map";
    // pc_map.header.stamp = ros::Time::now();
    // double large_y = 0;
    // double middle_x;
    // for (int i = 0; i < pc_without_QR_.points.size(); ++i)
    // {
    //   input.position.x = pc_without_QR_.points[i].x;
    //   input.position.y = pc_without_QR_.points[i].y;
    //   geometry_msgs::Pose output;
    //   world2map(input, output);
    //   geometry_msgs::Point32 p;
    //   p.x = output.position.x;
    //   p.y = output.position.y;
    //   if (fabs(large_y) < fabs(p.y)) {large_y = p.y; middle_x = p.x;}
    //   pc_map.points.push_back(p);
    // }

    // line_map.publish(pc_map);

    // tf::TransformListener listener;
    // tf::StampedTransform transform;

    // std::string target_frame = "map";
    // std::string source_frame = "world";

    // ros::Time time = ros::Time(0);

    // try {
    //     listener.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
    //     listener.lookupTransform(target_frame, source_frame, time, transform);
    // } catch (tf::TransformException &ex) {
    //     ROS_ERROR("%s", ex.what());
    //     ros::Duration(1.0).sleep();
    // }

    // ROS_INFO_STREAM("Translation: " << transform.getOrigin().x() << ", " 
    //                 << transform.getOrigin().y() << ", " 
    //                 << transform.getOrigin().z());
    // ROS_INFO_STREAM("Rotation: " << transform.getRotation().x() << ", " 
    //                 << transform.getRotation().y() << ", " 
    //                 << transform.getRotation().z() << ", " 
    //                 << transform.getRotation().w());

    loop_rate.sleep();
    ros::spinOnce();
  }
}






int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "image_shower");
    Visualize MyVisualize;
    MyVisualize.Execute();

  return 0;
}

#endif