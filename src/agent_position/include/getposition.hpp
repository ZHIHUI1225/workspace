
#ifndef LIB_VISULIZE_H
#define LIB_VISULIZE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "nav_msgs/Odometry.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Point.h>
#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cfloat>
#include <fstream>
#include <ctime>
#include <string>
#include <bits/stdc++.h>

#include <list>
#include <math.h>
#include <assert.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Eigen>
#include <opencv2/aruco.hpp>
#include <tf/transform_listener.h>
#include <robot_msg/robot_pose.h>
#include <robot_msg/robot_pose_array.h>
#include <robot_msg/target_pose.h>
#include <robot_msg/target_pose_array.h>
#include "skeletonization.hpp"


using namespace std;
using namespace cv;
int Kernel_size = 15;
int low_threshold = 40;
int high_threshold = 120;
std::vector<int> markerIds;
class Visualize //速度控制类
{
    public:
    Visualize();
    ~Visualize();
    /** Functions **/
    /**
     * @brief 管理unitree_nav_node节点的运行
     * 程序运行
     */
    void Execute();

    private:
    /** Const **/
    const int ROS_RATE_HZ = 50; //ROS更新频率

    /** Node Handle **/
    ros::NodeHandle n; // 节点句柄

	ros::Subscriber image_sub;
	ros::Publisher robot_pub_,line_without_QR_pub,Red_points_pub,obstacle_pub_ ,Red_points34_pub;
	robot_msg::robot_pose robot_;
	// sensor_msgs::PointCloud poly_pc_;
	
	inline void initializeQuaternion(geometry_msgs::Quaternion& quaternion)
		{
			quaternion.x = 0;
			quaternion.y = 0;
			quaternion.z = 0;
			quaternion.w = 1;
		}
	//多项式拟合
	bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
	{
	//Number of key points
	int N = key_point.size();
	//构造矩阵X
	cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
	for (int j = 0; j < n + 1; j++)
	{
	for (int k = 0; k < N; k++)
	{
	X.at<double>(i, j) = X.at<double>(i, j) +
	std::pow(key_point[k].x, i + j);
	}
	}
	}
	//构造矩阵Y
	cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
	for (int k = 0; k < N; k++)
	{
	Y.at<double>(i, 0) = Y.at<double>(i, 0) +
	std::pow(key_point[k].x, i) * key_point[k].y;
	}
	}
	A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	//求解矩阵A
	cv::solve(X, Y, A, cv::DECOMP_LU);
	return true;
	}
	// generate the circular mask
	cv::Mat extractCircularMask(cv::Mat img, Point center, int r) 
	{
    cv::Mat cirMask = img.clone();
    cirMask.setTo(cv::Scalar::all(255));
    cv::circle(cirMask, center, r, cv::Scalar(0, 0, 0), -1, 8, 0);
    return cirMask;
	}
	void imageCallBack(const sensor_msgs::Image::ConstPtr& Input)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(Input, sensor_msgs::image_encodings::BGR8);
		//680 480
		//cv::Mat frame= cv_ptr->image(cv::Range(30, 360), cv::Range(20, 640));
		cv::Mat frame_cut= cv_ptr->image(cv::Range::all(), cv::Range::all());
       
	// //detect Aruco
	// 	// std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

		cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<
			1.18346606e+03, 0.0, 3.14407234e+02,
			0.0, 1.18757422e+03, 2.38823696e+02,
			0.0, 0.0, 1);

		double fx, fy, cx, cy, k1, k2, k3, p1, p2;
		// fx = 1.18346606e+03;
		// fy = 1.18757422e+03;
		// cx = 3.14407234e+02;
		// cy = 2.38823696e+02;
		// k1 = -0.51328742;
		// k2 = 0.33232725;
		// p1 = 0.01683581;
		// p2 = -0.00078608;
		// k3 = -0.1159959;
		fx= 2.3003531127160736e+03;//RGB 
		cx= 9.5950000000000000e+02;
		fy= 2.3003531127160736e+03;
		cy= 5.3950000000000000e+02;
		k1 = 3.5007023317793756e-01;
		k2= -3.7943939452733586e+00;
		p1= 0;
		p2= 0;
		k3 = 8.0266257615514434e+00;
		cv::Mat distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
		std::vector<cv::Vec3d> rvecs, tvecs;
	
	cv::Mat gray,thre;
	cv::cvtColor(frame_cut, gray, cv::COLOR_BGR2GRAY);
	// Mat out;
	// //获取自定义核
	// Mat element = getStructuringElement(MORPH_RECT, Size(100, 100)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
	// //膨胀操作
	// dilate(gray, out, element);
	//二值
	threshold(gray, thre, 100, 255, THRESH_BINARY);
	// medianBlur(gray, gray, 5);
	// imshow("thre",thre);
	Mat Mask1 = Mat::zeros(frame_cut.size(), CV_8UC1); //】】掩膜
	robot_msg::robot_pose_array robot_array_new, obstacle_array;
	cv::aruco::detectMarkers(frame_cut, dictionary, markerCorners, markerIds);
	cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
	double deltax;
	double deltay;
	double xnew;
	double ynew;
	double robotmatrix[4][2];
		if (markerIds.size() > 0) 
		{	cv::aruco::drawDetectedMarkers(frame_cut, markerCorners, markerIds);
			for (int i = 0; i < rvecs.size(); ++i) {
				//cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
				robot_msg::robot_pose robot_new;			
				robot_new.ID.data = markerIds[i];
				xnew=(markerCorners[i][0].x + markerCorners[i][1].x + markerCorners[i][2].x + markerCorners[i][3].x)/4;
				ynew=(markerCorners[i][0].y + markerCorners[i][1].y + markerCorners[i][2].y + markerCorners[i][3].y)/4;
				
				if (markerIds[i]>10 && markerIds[i]<15){
					//读取圆心
					// deltax=20+(44-robot_new.position.x)/25.65;
					// deltay=7+(32-robot_new.position.y)/29.71;
					deltax=20+(44-xnew)/25.65;
					deltay=7+(32-ynew)/29.71;
					robot_new.position.x = xnew+deltax;
					robot_new.position.y = ynew+deltay;
					robot_new.yaw = atan2(markerCorners[i][0].y - markerCorners[i][3].y, markerCorners[i][0].x - markerCorners[i][3].x); // atan2(markerCorners[i][1].y - markerCorners[i][0].y, markerCorners[i][1].x - markerCorners[i][0].x)
					robot_array_new.robot_pose_array.push_back(robot_new);
					robotmatrix[markerIds[i]-11][0]=robot_new.position.x ;
					robotmatrix[markerIds[i]-11][1]=robot_new.position.y ;
					Point center(cvRound(robot_new.position.x ), cvRound(robot_new.position.y));
					//读取半径
					int radius = cvRound(40);
					//绘制圆
					circle(frame_cut, center, radius, Scalar(0, 0, 255), 1, 8, 0);
					circle(Mask1, center, radius, Scalar(255, 255, 255), -1); 
					}
					if (markerIds[i]<10 && markerIds[i]>5){
					//读取圆心
					// deltax=8-16/1062*(xnew-66);
					deltax=8-(xnew-43)/58.375;
					deltay=7+(32-ynew)/29.71;
					robot_new.position.x = xnew+deltax;
					robot_new.position.y = ynew+deltay;
					robot_new.yaw = atan2(markerCorners[i][0].y - markerCorners[i][3].y, markerCorners[i][0].x - markerCorners[i][3].x); // atan2(markerCorners[i][1].y - markerCorners[i][0].y, markerCorners[i][1].x - markerCorners[i][0].x)
					robot_array_new.robot_pose_array.push_back(robot_new);
					Point center(cvRound(robot_new.position.x ), cvRound(robot_new.position.y));
					//读取半径
					int radius = cvRound(32);
					// cv::imshow("cicle",frame_cut);
					//绘制圆
					circle(frame_cut, center, radius, Scalar(0, 0, 255), 1, 8, 0);
					circle(Mask1, center, radius, Scalar(255, 255, 255), -1); 
					}	
					if (markerIds[i]>0 && markerIds[i]<6){
						robot_msg::robot_pose obstacle;
						obstacle.ID.data = markerIds[i];
						if (markerIds[i]==1){
							deltax=8-(xnew-43)/58.375;
							deltay=7+(32-ynew)/29.71;
							obstacle.position.x = xnew+deltax;
							obstacle.position.y = ynew+deltay;
							int radius = cvRound(60);
							// cv::imshow("cicle",frame_cut);
							//绘制圆
							Point center(cvRound(obstacle.position.x ), cvRound(obstacle.position.y));
							circle(frame_cut, center, radius, Scalar(0, 0, 255), 1, 8, 0);
							circle(Mask1, center, radius, Scalar(255, 255, 255), -1); 
						}
						else
						{
							
							obstacle.position.x=markerCorners[i][0].x ;
							obstacle.position.y=markerCorners[i][0].y ;
							obstacle_array.robot_pose_array.push_back(obstacle);
							Point center(cvRound(obstacle.position.x ), cvRound(obstacle.position.y));//读取半径
							int radius = cvRound(2);
							// cv::imshow("cicle",frame_cut);
							//绘制圆
							circle(frame_cut, center, radius, Scalar(255, 0, 255), -1);
							Point center1(cvRound(xnew), cvRound(ynew));
							radius = cvRound(20);
							circle(Mask1, center1, radius, Scalar(255, 255, 255), -1); 
						}
						
					}
			}
		}
	// Point center(500,200);
	// circle(frame_cut, center, 35, Scalar(0, 255, 255), 1,8,0); 
	robot_pub_.publish(robot_array_new);
	obstacle_pub_.publish(obstacle_array);
	
	//give the mask of the system to detetct red points
	cv::Mat mask12 = cv::Mat::zeros(frame_cut.rows, frame_cut.cols, CV_8UC1);
	double Ke;
	double aa=100;
	double rx1,ry1,rx2,ry2;
	if (robotmatrix[1][1]==robotmatrix[0][1])
	{
		rx1=robotmatrix[1][0];
		rx2=robotmatrix[0][0];
		if (robotmatrix[1][0]>robotmatrix[0][0])
		{
			
			ry1=robotmatrix[1][1]-aa;
			ry2=-aa+robotmatrix[0][1];
		}
		else{
			ry1=robotmatrix[1][1]+aa;
			ry2=aa+robotmatrix[0][1];
		}
	}
	else{
		Ke=-(robotmatrix[1][0]-robotmatrix[0][0])/(robotmatrix[1][1]-robotmatrix[0][1]);
	if (robotmatrix[1][1]>=robotmatrix[0][1]){
		rx1=aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[1][0];
		rx2=aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[0][0];
		
	}
	else
	{
		rx1=-aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[1][0];
		rx2=-aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[0][0];
		
	}
	ry1=robotmatrix[1][1]+Ke*(rx1-robotmatrix[1][0]);
	ry2=Ke*(rx2-robotmatrix[0][0])+robotmatrix[0][1];
	}
    // Define the polygon points
    std::vector<cv::Point> points = {cv::Point(robotmatrix[0][0], robotmatrix[0][1]), cv::Point(robotmatrix[1][0], robotmatrix[1][1]), cv::Point(rx1, ry1), cv::Point(rx2, ry2)};
    std::vector<std::vector<cv::Point>> all_points = {points};

    // Draw the polygon on the mask
    cv::fillPoly(mask12, all_points, cv::Scalar(255));

    // Apply the mask on the image
    cv::Mat masked_image;
    cv::bitwise_and(frame_cut, frame_cut, masked_image, mask12 = mask12);
	cv::imshow("Masked Image", masked_image);

	// red color detection

	Mat imgAddMask = masked_image.clone();
	imgAddMask .setTo(255, Mask1);
    cv::Mat hsv_frame;
    cv::cvtColor(imgAddMask, hsv_frame, cv::COLOR_BGR2HSV);
	
    cv::Scalar low_red = cv::Scalar(0,45, 20);
    cv::Scalar high_red = cv::Scalar(20, 200, 200);

    cv::Mat red_mask;
    cv::inRange(hsv_frame, low_red, high_red, red_mask);

    cv::Mat red;
    cv::bitwise_and(frame_cut,frame_cut, red, red_mask);
	sensor_msgs::PointCloud red_points;
	for (int ro = 0; ro < red.rows; ro++)
	{
		for (int co = 0; co <red.cols; co++)
		{
			
			Vec3b& color=red.at<Vec3b>(ro, co);
			if (color[1]>low_red[1]-1)
			{
				geometry_msgs::Point32 p;
				p.x=co;
				p.y=ro;
				p.z=0;
				red_points.points.push_back(p);	
			}
			
		}
	}
for (auto point : red_points.points) {
    circle(red, Point(point.x, point.y), 1, Scalar(255, 255, 255), -1);
}

	Red_points_pub.publish(red_points);
	// cv::imshow("red color",red);

// robot 34  red points detect
//give the mask of the system to detetct red points
	cv::Mat mask34 = cv::Mat::zeros(frame_cut.rows, frame_cut.cols, CV_8UC1);
	if (robotmatrix[3][1]==robotmatrix[2][1])
	{
		rx1=robotmatrix[3][0];
		rx2=robotmatrix[2][0];
		if (robotmatrix[3][0]>robotmatrix[2][0])
		{
			
			ry1=robotmatrix[3][1]-aa;
			ry2=-aa+robotmatrix[2][1];
		}
		else{
			ry1=robotmatrix[3][1]+aa;
			ry2=aa+robotmatrix[2][1];
		}
	}
	else{
		Ke=-(robotmatrix[3][0]-robotmatrix[2][0])/(robotmatrix[3][1]-robotmatrix[2][1]);
	if (robotmatrix[3][1]>=robotmatrix[2][1]){
		rx1=aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[3][0];
		rx2=aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[2][0];
	}
	else
	{
		rx1=-aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[3][0];
		rx2=-aa*std::sqrt(1/(1+Ke*Ke))+robotmatrix[2][0];
	}
	ry1=robotmatrix[3][1]+Ke*(rx1-robotmatrix[3][0]);
	ry2=Ke*(rx2-robotmatrix[2][0])+robotmatrix[2][1];
	}
    // Define the polygon points
    std::vector<cv::Point> points34 = {cv::Point(robotmatrix[2][0], robotmatrix[2][1]), cv::Point(robotmatrix[3][0], robotmatrix[3][1]), cv::Point(rx1, ry1), cv::Point(rx2, ry2)};
    std::vector<std::vector<cv::Point>> all_points34 = {points34};

    // Draw the polygon on the mask
    cv::fillPoly(mask34, all_points34, cv::Scalar(255));

    // Apply the mask on the image
    cv::Mat masked_image34;
    cv::bitwise_and(frame_cut, frame_cut, masked_image34, mask34 = mask34);
	// cv::imshow("Masked Image34", masked_image34);
	// red color detection

	Mat imgAddMask34 = masked_image34.clone();
	imgAddMask34 .setTo(255, Mask1);
  
    cv::cvtColor(imgAddMask34, hsv_frame, cv::COLOR_BGR2HSV);

    cv::Mat red_mask34;
    cv::inRange(hsv_frame, low_red, high_red, red_mask34);

    cv::Mat red34;
    cv::bitwise_and(frame_cut,frame_cut, red34, red_mask34);
	sensor_msgs::PointCloud red_points34;
	for (int ro = 0; ro < red34.rows; ro++)
	{
		for (int co = 0; co <red34.cols; co++)
		{
			
			Vec3b& color=red34.at<Vec3b>(ro, co);
			if (color[1]>low_red[1]-1)
			{
				geometry_msgs::Point32 p;
				p.x=co;
				p.y=ro;
				p.z=0;
				red_points34.points.push_back(p);	
			}
			
		}
	}
for (auto point : red_points34.points) {
    circle(red34, Point(point.x, point.y), 1, Scalar(255, 255, 255), -1);
}
	Red_points34_pub.publish(red_points34);
	// cv::imshow("red color34",red34);

	Mat imgAddMask1 = thre.clone();
	cv::add(Mask1,thre,imgAddMask1);

	//逆转二值图
	Mat thre_inv;
	bitwise_not(imgAddMask1,thre_inv);
		imshow("points",frame_cut);
		waitKey(3);
	}
	void world2map(geometry_msgs::Pose& input,
                  geometry_msgs::Pose& output,
					tf::TransformListener& listener)
	{	
	    initializeQuaternion(input.orientation);
		geometry_msgs::PoseStamped input_s, output_s;
		input_s.header.frame_id = "world";
		input_s.pose = input;
		output_s.header.frame_id = "map";
    // tf::StampedTransform transform;

    std::string target_frame = "map";
    std::string source_frame = "world";

    ros::Time time = ros::Time(0);

    try {
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
        //listener.lookupTransform(target_frame, source_frame, time, transform);
		listener.transformPose("map",input_s,output_s);
		output = output_s.pose;
    } 
	catch (tf::TransformException &ex) {
        ROS_ERROR("Failed to transform pose: %s", ex.what());
        // ros::Duration(1.0).sleep();
    }
		// return true;
	}
void world2local(geometry_msgs::Pose& input,
                geometry_msgs::Pose& output,
				tf::TransformListener& listener)
	{	
	   initializeQuaternion(input.orientation);
		geometry_msgs::PoseStamped input_s, output_s;
		input_s.header.frame_id = "world";
		input_s.pose = input;
		output_s.header.frame_id = "local";
		
	// tf::TransformListener listener;
    // tf::StampedTransform transform;

    std::string target_frame = "local";
    std::string source_frame = "world";

    ros::Time time = ros::Time(0);

    try {
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
        //listener.lookupTransform(target_frame, source_frame, time, transform);
		listener.transformPose("local",input_s,output_s);
		output = output_s.pose;
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
	}


}; // Visualize

#endif // LIB_VISULIZE_H

