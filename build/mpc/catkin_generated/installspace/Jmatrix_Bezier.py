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
from scipy.special import comb

wheel_base = 80e-3  # mm
wheel_diameter = 31e-3  # mm
L=280e-3 #the length of tube
def get_bezier_parameters(X, Y, degree=3):
    """ Least square qbezier fit using penrose pseudoinverse.
    """
    if degree < 1:
        raise ValueError('degree must be 1 or greater.')

    if len(X) != len(Y):
        raise ValueError('X and Y must be of the same length.')

    if len(X) < degree + 1:
        raise ValueError(f'There must be at least {degree + 1} points to '
                         f'determine the parameters of a degree {degree} curve. '
                         f'Got only {len(X)} points.')

    def bpoly(n, t, k):
        """ Bernstein polynomial when a = 0 and b = 1. """
        return t ** k * (1 - t) ** (n - k) * comb(n, k)
        #return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bmatrix(T):
        """ Bernstein matrix for Bézier curves. """
        return np.matrix([[bpoly(degree, t, k) for k in range(degree + 1)] for t in T])

    def least_square_fit(points, M):
        M_ = np.linalg.pinv(M)
        return M_ * points

    T = np.linspace(0, 1, len(X))
    M = bmatrix(T)
    points = np.array(list(zip(X, Y)))
    
    final = least_square_fit(points, M).tolist()
    final[0] = [X[0], Y[0]]
    final[len(final)-1] = [X[len(X)-1], Y[len(Y)-1]]
    return final

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=50):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals


class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.controlpoints=np.zeros((1,10))
        # self.sub = rospy.Subscriber('/line_without_QR', PointCloud, self.tube_callback,queue_size=10)
        self.sub = rospy.Subscriber('control_points',PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        i=0
        for pose in msg.poses:
            self.controlpoints[0][i]=pose.position.x* 1.5037594e-3
            self.controlpoints[0][i+1]=pose.position.y* 1.5306122e-3
            i=i+2

class QRrobot:
    def __init__(self,N):
        self.robotx=[0.0]*N
        self.roboty=[0.0]*N
        self.robotyaw=[0.0]*N
        self.robotID=[0]*N
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
        w1[i,0]= (uLinear1 + uAngular1 * wheel_base / 2) * 2 / wheel_diameter
        w1[i,1]= (uLinear1 - uAngular1 * wheel_base / 2) * 2 / wheel_diameter
        w2[i,0]= (uLinear2 + uAngular2 * wheel_base / 2) * 2 / wheel_diameter
        w2[i,1]= (uLinear2 - uAngular2 * wheel_base / 2) * 2 / wheel_diameter       
        w[i,:]=[w1[i,0],w1[i,1],w2[i,0],w2[i,1]]
    return w

if __name__ == '__main__':
    try:
        rospy.init_node('Jmatrix_update')
        N=5
        Robot = QRrobot(N)
        feature=Point_tube()
        pub = rospy.Publisher('anglevelocity', Float64MultiArray, queue_size=10)
        vel = [0]*2
        rate = rospy.Rate(5)
        p=[Robot.robotx[0],Robot.roboty[0],Robot.robotx[1],Robot.roboty[1]] #position of robot
        # tp=[feature.middlepoint.x,feature.middlepoint.y]
        tp=feature.controlpoints[0][2:-2]
        x=[]
        y=[]
        i=0
        T_num=70
        while not rospy.is_shutdown():
            if i <T_num and Robot.flag==1 and feature.controlpoints[0][0]!=0:
                #random move
                if i==0:
                    tp=feature.controlpoints[0][2:-2].copy()
                # ran_vel=0.4*np.random.uniform(-1, 1, size=(1, 4))
                u_sol=np.array([[0.03*random.random(),0.8*random.uniform(-1, 1),0.03*random.random(),0.8*random.uniform(-1, 1)]])
                ran_vel=v2w(u_sol,1)
                vel_msg = Float64MultiArray(data=ran_vel[0])
                rospy.loginfo(vel_msg)
                pub.publish(vel_msg)
                d = rospy.Duration(0.3)
                rospy.sleep(d)
                ran_vel=np.zeros((1,4))
                vel_msg = Float64MultiArray(data=ran_vel[0])
                rospy.loginfo(vel_msg)
                pub.publish(vel_msg)
                d = rospy.Duration(0.2)
                rospy.sleep(d)
                p_new=[Robot.robotx[0],Robot.roboty[0],Robot.robotx[1],Robot.roboty[1]]
                tp_new=feature.controlpoints[0][2:-2].copy()
                deltax=np.array(p_new)-np.array(p)
                deltay=tp_new-tp
                y.append(np.reshape(deltay,(1,2*(N-2)))[0])
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
                    d = rospy.Duration(0.3)
                    rospy.sleep(d)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
