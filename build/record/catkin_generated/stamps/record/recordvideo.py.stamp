#!/usr/bin/env python -m memory_profiler
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from robot_msg.msg import robot_pose_array
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from scipy.special import comb
import numpy as np
from cv_bridge import CvBridge
import cv2
import math
import rospy
import tf
import time
import matplotlib.pyplot as plt                                 # TF坐标变换库
from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from numpy import linalg as LA
import scipy.optimize as opt

class frame_image():
    def __init__(self):
        # Params
        self.image=None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(50)
        # Subscribers
        rospy.Subscriber('/camera/image',Image,self.image_callback,queue_size=10)

    def image_callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        # cv2.imshow("image_tf",self.image)
        # cv2.waitKey(3)

class Targetzone():
    def __init__(self):
        self.target=Point32()
        self.flag=0
        rospy.Subscriber('/Targetzone',Point32,self.Callback,queue_size=10)
    def Callback(self,msg):
        self.target=msg
        self.flag=1

if __name__ == '__main__':
    try:
        rospy.init_node('record')
        Frame=frame_image()
        Targe=Targetzone()
        rate = rospy.Rate(60.0)

        flag=0
        while not rospy.is_shutdown() :
            if Frame.image is not None and flag==0:
                wri = cv2.VideoWriter('record.avi', cv2.VideoWriter_fourcc(*'XVID'), 60, (1229-63,520-44), True)
                flag=1
            if flag==1 and Frame.image is not None:
                if Targe.flag==1:
                    P=(int(Targe.target.x),int(Targe.target.y))
                    cv2.circle(Frame.image, P, int(Targe.target.z), (0, 0, 255), -1)
                wri.write(Frame.image)
                cv2.imshow("frame",Frame.image)
                cv2.waitKey(1)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    wri.release()