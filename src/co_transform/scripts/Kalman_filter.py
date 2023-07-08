import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from robot_msg.msg import robot_pose_array
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
from scipy.special import comb
import numpy as np
import cv2
import math
import casadi as ca
import casadi.tools as ca_tools
import time
import matplotlib.pyplot as plt
import csv
from std_msgs.msg import Float64MultiArray
from scipy.optimize import fsolve
from scipy.special import ellipe
from cv_bridge import CvBridge
import cv2
from numpy import linalg as LA
import scipy.optimize as opt

N_target=3 # feature points number

class frame_image():
    def __init__(self):
        # Params
        self.image=None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)
        # Subscribers
        rospy.Subscriber('/camera/image',Image,self.image_callback,queue_size=10)
    def image_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)


class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.controlpoints=np.zeros((1,10))
        self.flag=0
        # self.sub = rospy.Subscriber('/line_without_QR', PointCloud, self.tube_callback,queue_size=10)
        self.sub = rospy.Subscriber('control_points',PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        i=0
        for pose in msg.poses:
            self.controlpoints[0][i]=pose.position.x
            self.controlpoints[0][i+1]=pose.position.y
            i=i+2
        self.flag=1


'''KALMAN FILTERING CLASS FOR N 2D POINTS'''
'''Kalman filtering for selected points in an image using OpenCV cv2.kalmanFilter class in Python '''

class Kalman_Filtering_2D:

    def __init__(self, n_points):
        self.n_points = n_points

    def initialize(self):

        n_states = self.n_points * 4
        n_measures = self.n_points * 2
        self.kalman = cv2.KalmanFilter(n_states, n_measures)
        kalman = self.kalman
        kalman.transitionMatrix = np.eye(n_states, dtype=np.float32)
        #kalman.processNoiseCov = np.eye(n_states, dtype = np.float32)*0.9
        kalman.measurementNoiseCov = np.eye(
            n_measures, dtype=np.float32)*0.0005

        kalman.measurementMatrix = np.zeros((n_measures, n_states), np.float32)
        dt = 1

        self.Measurement_array = []
        self.dt_array = []

        for i in range(0, n_states, 4):
            self.Measurement_array.append(i)
            self.Measurement_array.append(i+1)

        for i in range(0, n_states):
            if i not in self.Measurement_array:
                self.dt_array.append(i)

        # Transition Matrix for [x,y,x',y'] for n such points
        # format of first row [1 0 dt 0 .....]
        for i, j in zip(self.Measurement_array, self.dt_array):
            kalman.transitionMatrix[i, j] = dt

        # Measurement Matrix for [x,y,x',y'] for n such points
        # format of first row [1 0 0 0 .....]
        for i in range(0, n_measures):
            kalman.measurementMatrix[i, self.Measurement_array[i]] = 1

    def predict(self, points):

        pred = []
        input_points = np.float32(np.ndarray.flatten(points))
        # Correction Step
        self.kalman.correct(input_points)
        # Prediction step
        tp = self.kalman.predict()

        for i in self.Measurement_array:
            pred.append(float(tp[i]))

        return pred


'''
USAGE: points must be a 2d numpy array of points, e.g.
input points are:
[[ x1.  y1.]
 [ x2.  y2.]
 [ x3.  y3.]
 [ x4.  y4.]
 [ x5.  y5.]
 [ x6.  y6.]]
import kalman_class
kf = kalman_class.Kalman_Filtering(6)
kf.initialize()
...
...
kf.predict(points)
'''

if  __name__ == '__main__':
    try:
        rospy.init_node('Kalman')
        feature=Point_tube()
        Frame=frame_image()
        kalman=Kalman_Filtering_2D(N_target)
        kalman.initialize()
        while not rospy.is_shutdown():
            if feature.flag==1 and Frame.image is not None:
                xt=feature.controlpoints[0]
                xt=np.reshape(xt,(len(xt),1))
                xt=np.vstack((xt[2:-2]))
                points=xt.reshape(-1,2)
               
                for i in range(10):
                    points_pred=kalman.predict(points)

                for i in range(N_target+2):
                    center=(int(feature.controlpoints[0][2*i]),int(feature.controlpoints[0][2*i+1]))
                    cv2.circle(Frame.image, center,2, (0, 0, 255), -1)

                for i in range(N_target):
                    center1=(int(points_pred[2*i]),int(points_pred[2*i+1]))
                    cv2.circle(Frame.image, center1,2, (255, 0, 0), -1)
                cv2.imshow("Kalman",Frame.image)
                cv2.waitKey(1)
    except rospy.ROSInterruptException:
        pass