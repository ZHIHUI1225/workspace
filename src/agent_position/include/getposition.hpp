
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
	ros::Publisher robot_pub_,line_without_QR_pub,Red_points_pub;
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
		fx = 1.18346606e+03;
		fy = 1.18757422e+03;
		cx = 3.14407234e+02;
		cy = 2.38823696e+02;
		k1 = -0.51328742;
		k2 = 0.33232725;
		p1 = 0.01683581;
		p2 = -0.00078608;
		k3 = -0.1159959;
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
	robot_msg::robot_pose_array robot_array_new;
	cv::aruco::detectMarkers(frame_cut, dictionary, markerCorners, markerIds);
	cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
	double deltax;
	double deltay;
		if (markerIds.size() > 0) 
		{	cv::aruco::drawDetectedMarkers(frame_cut, markerCorners, markerIds);
			for (int i = 0; i < rvecs.size(); ++i) {
				//cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
				robot_msg::robot_pose robot_new;			
				robot_new.ID.data = markerIds[i];
				robot_new.position.x = (markerCorners[i][0].x + markerCorners[i][1].x + markerCorners[i][2].x + markerCorners[i][3].x)/4;
				robot_new.position.y = (markerCorners[i][0].y + markerCorners[i][1].y + markerCorners[i][2].y + markerCorners[i][3].y)/4;
				robot_new.yaw = atan2(markerCorners[i][0].y - markerCorners[i][3].y, markerCorners[i][0].x - markerCorners[i][3].x); // atan2(markerCorners[i][1].y - markerCorners[i][0].y, markerCorners[i][1].x - markerCorners[i][0].x)
				robot_array_new.robot_pose_array.push_back(robot_new);
				if (markerIds[i]>10 && markerIds[i]<13){
					//读取圆心
					deltax=20+(44-robot_new.position.x)/25.65;
					deltay=7+(32-robot_new.position.y)/29.71;
					Point center(cvRound(robot_new.position.x +deltax), cvRound(robot_new.position.y+deltay));
					//读取半径
					int radius = cvRound(38);
					// // cv:: Mat circlemask=extractCircularMask(gray,center, radius) ;
					// // Mat imgAddMask = gray.clone();
					// // cv::add(circlemask,gray,imgAddMask);
					// // cv::imshow("imgAddMask",imgAddMask);
					// vector<Vec3f> circles;
    				// HoughCircles(thre, circles, HOUGH_GRADIENT, 1,
                	//  70, 100, 90, 20, 40);
					// for( size_t i = 0; i < circles.size(); i++ )
					// 	{
					// 		Vec3i c = circles[i];
					// 		Point center = Point(c[0], c[1]);
					// 		// circle center
					// 		circle(frame_cut, center, 1, Scalar(0,100,100), 3, LINE_AA);
					// 		// circle outline
					// 		int radius = c[2];
					// 		circle(frame_cut, center, radius, Scalar(255,0,255), 3, LINE_AA);
					// 		circle(Mask1, center, radius, Scalar(255, 255, 255), -1); 
					// 	}
					//绘制圆
					circle(frame_cut, center, radius, Scalar(0, 0, 255), 1, 8, 0);
					circle(Mask1, center, radius, Scalar(255, 255, 255), -1); 
					}
					if (markerIds[i]<10){
					//读取圆心
					deltax=20+(44-robot_new.position.x)/25.65;
					deltay=7+(32-robot_new.position.y)/29.71;
					Point center(cvRound(robot_new.position.x +deltax), cvRound(robot_new.position.y+deltay));
					//读取半径
					int radius = cvRound(28);
					// cv::imshow("cicle",frame_cut);
					//绘制圆
					circle(frame_cut, center, radius, Scalar(0, 0, 255), 1, 8, 0);
					circle(Mask1, center, radius, Scalar(255, 255, 255), -1); 
					}	
			}
		}
	Point center(500,200);
	circle(frame_cut, center, 35, Scalar(0, 255, 255), 1,8,0); 
	robot_pub_.publish(robot_array_new);
	Mat imgAddMask = frame_cut.clone();
	imgAddMask .setTo(255, Mask1);

	// red color detection
    cv::Mat hsv_frame;
    cv::cvtColor(imgAddMask, hsv_frame, cv::COLOR_BGR2HSV);
	
    cv::Scalar low_red = cv::Scalar(0,45, 10);
    cv::Scalar high_red = cv::Scalar(20, 150, 150);

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
	cv::imshow("red color",red);

	Mat imgAddMask1 = thre.clone();
	cv::add(Mask1,thre,imgAddMask1);
	//cv::imshow("multiply", imgAddMask1);
	//逆转二值图
	Mat thre_inv;
	bitwise_not(imgAddMask1,thre_inv);
	// imshow("thre_inv",thre_inv);
	//连通域
	// RNG rng(10086);
	// Mat outr;
	// int number = connectedComponents(thre_inv, outr, 8, CV_16U);  //统计图像中连通域的个数

	// // 计算每个连通组件的大小
	// std::vector<int> sizes(number, 0);
	// for (int i = 0; i < outr.rows; i++) {
	// 	for (int j = 0; j < outr.cols; j++) {
	// 		int label = outr.at<uint16_t>(i, j);
		
	// 		if (label > 0) {
	// 			sizes[label] += 1;
	// 			//  cout<<"label"<<label<<endl;
	// 		}
	// 	}
	// }

	// // 找到最大的连通组件
	// int max_label = 1;
	// int max_size = sizes[1];
	// for (int i = 2; i <number; ++i) {
	// 	if (sizes[i] > max_size) {
	// 		max_label = i;
	// 		max_size = sizes[i];
	// 	}
	// }


	// Mat connect = Mat::zeros(thre.size(), frame_cut.type());
	// int w = connect.cols;
	// int h = connect.rows;
	// Vec3b color=Vec3b(rng.uniform(0,256),rng.uniform(0,256),rng.uniform(0,256));
	// // sensor_msgs::PointCloud pc_without_QR_;
	// for (int row = 0; row < h; row++)
	// {
	// 	for (int col = 0; col < w; col++)
	// 	{
	// 		int label = outr.at<uint16_t>(row, col);
	// 		geometry_msgs::Point32 p;
	// 		if (label == max_label)  
	// 		{
	// 			connect.at<Vec3b>(row, col)=color;
	// 			p.x=col;
	// 			p.y=row;
	// 			p.z=0;
	// 			// pc_without_QR_.points.push_back(p);	
	// 		}
	// 	}
	// }
	//   //骨干提取
    //   Mat skel = skeletonization(connect);
    // 	// imshow("Skeleton Image", skel);
    //     //曲线拟合
    //     //输入拟合点
    //     std::vector<cv::Point> points;
	// 	sensor_msgs::PointCloud pc_without_QR_;
    //     int rowmin=h;
    //     int colmin=w;
    //     int colmax=0;
    //     for (int col = 0; col < w; col++)
    //     {
    //         for (int row = 0; row < h; row++)
    //         {
    //             if (connect.at<Vec3b>(row, col) == color)  
	// 			// if (skel.at<uchar>(row, col) == 255) 
    //             {
	// 				geometry_msgs::Point32 p;
    //                 points.push_back(cv::Point(col, row));
	// 				p.x=col;
	// 				p.y=row;
	// 				p.z=0;
	// 				pc_without_QR_.points.push_back(p);
    //                 if (row<rowmin)
    //                 {
    //                     rowmin=row;
    //                 }
    //                 if(col<colmin)
    //                 {
    //                     colmin=col;
    //                 }
    //                 if(col>colmax)
    //                 {
    //                     colmax=col;
    //                 }
    //             }
                
    //         }
    //     }
	// 	line_without_QR_pub.publish(pc_without_QR_);
	// 	cv::Mat A;
    //     int Num=6;
    //     polynomial_curve_fit(points, Num, A);
        //std::cout << "A = " << A << std::endl;
        //计算曲线拟合的点
        // std::vector<cv::Point> points_fitted;
        // for (int i=0; i < 100; ++i)
        // {
        //     Point2d ipt;
        //     ipt.x = i*(colmax-colmin)/99+colmin;
        //     ipt.y = 0;
        //     for (int j = 0; j < Num + 1; ++j)
        //     {
        //     ipt.y +=A.at<double>(j, 0)*pow(ipt.x, j); // 计算拟合函数的值
        //     }
        //     points_fitted.push_back(ipt);
        //     // Point center(cvRound(points_fitted[i].x), cvRound(points_fitted[i].y));
        //     // cv::circle(frame, center, 3, cv::Scalar(0, 255, 255), -1);
        // }

		// int k=0;
		// std::vector<cv::Point>  pc_10;
		// for (int i = 1; i < points.size(); ++i)
		// {
		// double l = hypot(points[k].x - points[i].x, points[k].y - points[i].y);
		// if (l > 5.0) { 
		// 	Point2d ipt;
		// 	ipt.x=points[k].x;
		// 	ipt.y=points[k].y;
		// 	pc_10.push_back(ipt);
		// 	k = i;
		// 	Point center(cvRound(ipt.x), cvRound(ipt.y));
        //     // cv::circle(frame_cut, center, 3, cv::Scalar(0, 0, 255), -1);
		// }
		// }
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

