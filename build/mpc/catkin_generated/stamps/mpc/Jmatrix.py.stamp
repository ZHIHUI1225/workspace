#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from robot_msg.msg import robot_pose_array
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import math
import time
import random 
import matplotlib.pyplot as plt
import csv
from std_msgs.msg import Float64MultiArray
import time
from scipy.optimize import fsolve
from scipy.special import ellipe
from sklearn.linear_model import LinearRegression

wheel_base = 80e-3  # mm
wheel_diameter = 31e-3  # mm
L=300e-3 #the length of tube

class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.sub = rospy.Subscriber('/feature_points', PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        self.feature_point=PointCloud()
        for pose in msg.poses:
            self.feature_point.points.append(pose.position)
        if len(msg.poses)%2==1: # odd
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses)+1)/2)].x* 1.5037594e-3 #to m
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses)+1)/2)].y* 1.5306122e-3 
        else:
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses))/2)].x* 1.5037594e-3
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses))/2)].y* 1.5306122e-3 
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.poses)


class QRrobot:
    def __init__(self,image=None,x=None,y=None,xmax=None,ymax=None):
        self.robotx=[0.0]*2
        self.roboty=[0.0]*2
        self.robotyaw=[0.0]*2
        self.robotID=[0]*2
        self.flag=0
        self.sub = rospy.Subscriber('/robot', robot_pose_array, self.pose_callback,queue_size=10)
    def pose_callback(self, msg): # feedback means actual value.
        #print(len(msg.robot_pose_array))
        self.flag=0
        for i in range(len(msg.robot_pose_array)):
            ID=int(msg.robot_pose_array[i].ID.data-10)
            self.robotx[int(ID-1)]=msg.robot_pose_array[i].position.x* 1.5037594e-3
            self.roboty[int(ID-1)]=msg.robot_pose_array[i].position.y* 1.5306122e-3 
            self.robotID[int(ID-1)]=msg.robot_pose_array[i].ID.data
            self.robotyaw[int(ID-1)]=msg.robot_pose_array[i].yaw
        self.flag=1

def v2w(u_sol,N):
    w1=np.zeros((N,2))
    w2=np.zeros((N,2))
    w=np.zeros((N,4))
    for i in range(N):
        uLinear1=u_sol[i,0]
        uAngular1=u_sol[i,1]
        uLinear2=u_sol[i,2]
        uAngular2=u_sol[i,3]
        w1[i,0]= (uLinear1 - uAngular1 * wheel_base / 2) * 2 / wheel_diameter
        w1[i,1]= (uLinear1 + uAngular1 * wheel_base / 2) * 2 / wheel_diameter
        w2[i,0]= (uLinear2 - uAngular2 * wheel_base / 2) * 2 / wheel_diameter
        w2[i,1]= (uLinear2 + uAngular2 * wheel_base / 2) * 2 / wheel_diameter       
        w[i,:]=[w1[i,0],w1[i,1],w2[i,0],w2[i,1]]
    return w

if __name__ == '__main__':
    try:
        rospy.init_node('Jmatrix_update')
        Robot = QRrobot()
        feature=Point_tube()
        pub = rospy.Publisher('anglevelocity', Float64MultiArray, queue_size=10)
        vel = [0]*2
        rate = rospy.Rate(1)
        p=[Robot.robotx[0],Robot.roboty[0],Robot.robotx[1],Robot.roboty[1]] #position of robot
        tp=[feature.middlepoint.x,feature.middlepoint.y]
        x=[]
        y=[]
        i=0
        T_num=50
        while not rospy.is_shutdown():
            if i <T_num and Robot.flag==1 and feature.middlepoint is not None:
                #random move
                # ran_vel=0.4*np.random.uniform(-1, 1, size=(1, 4))
                u_sol=np.array([[0.02*random.random(),0.1*random.uniform(-1, 1),0.02*random.random(),0.1*random.uniform(-1, 1)]])
                ran_vel=v2w(u_sol,1)
                vel_msg = Float64MultiArray(data=ran_vel[0])
                rospy.loginfo(vel_msg)
                pub.publish(vel_msg)
                d = rospy.Duration(0.5)
                rospy.sleep(d)
                p_new=[Robot.robotx[0],Robot.roboty[0],Robot.robotx[1],Robot.roboty[1]]
                tp_new=[feature.middlepoint.x,feature.middlepoint.y]
                deltax=np.array(p_new)-np.array(p)
                deltay=np.array(tp_new)-np.array(tp)
                y.append(np.reshape(deltay,(1,2))[0])
                x.append(deltax)
                p=p_new
                tp=tp_new
                i=i+1
                if i>T_num-10:
                # Find the coefficients (m and b) of the line y = mx + b that best fits the data
                    model = LinearRegression().fit(x, y)
                    print(model.coef_)
                if i==T_num:
                    ran_vel=np.zeros((1,4))
                    vel_msg = Float64MultiArray(data=ran_vel[0])
                    rospy.loginfo(vel_msg)
                    pub.publish(vel_msg)
                    d = rospy.Duration(0.5)
                    rospy.sleep(d)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
