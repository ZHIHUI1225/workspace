#!/usr/bin/env python -m memory_profiler
from std_msgs.msg import Int8
from std_msgs.msg import Bool
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

class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.sub = rospy.Subscriber('/feature_points', PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        self.feature_point=PointCloud()
        for pose in msg.poses:
            pose.position.x= pose.position.x
            pose.position.y= pose.position.y
            self.feature_point.points.append(pose.position)
        if len(msg.poses)%2==1: # odd
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses)+1)/2)].x
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses)+1)/2)].y
        else:
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses))/2)].x
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses))/2)].y
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.poses)

class QRrobot:
    def __init__(self):
        N=10
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
            if msg.robot_pose_array[i].ID.data>10:
                ID=int(msg.robot_pose_array[i].ID.data-10)
            else:
                ID=int(msg.robot_pose_array[i].ID.data)
            self.robotx[int(ID-1)]=msg.robot_pose_array[i].position.x* 1.5037594e-3
            self.roboty[int(ID-1)]=msg.robot_pose_array[i].position.y* 1.5306122e-3 
            self.robotID[int(ID-1)]=ID
            self.robotyaw[int(ID-1)]=msg.robot_pose_array[i].yaw
        self.flag=1

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

class FLAG():
    def __init__(self):
        self.enclose_flag=False
        self.transport_flag=False
        self.enclose_sub=rospy.Subscriber('encloseflag',Bool,self.enclose_flag_callback,queue_size=10)
        self.transport_sub=rospy.Subscriber('transportflag',Bool,self.transport_flag_callback,queue_size=10)
    def enclose_flag_callback(self,msg):
        self.enclose_flag=msg.data
    def transport_flag_callback(self,msg):
        self.transport_flag=msg.data


if __name__ == '__main__':
    try:
        rospy.init_node('record')
        Frame=frame_image()
        Robot = QRrobot()
        F=FLAG()
        feature=Point_tube()
        TargetID_pub=rospy.Publisher('TargetID',Int8,queue_size=10)
        mpc1_flag_pub=rospy.Publisher('goflag',Bool,queue_size=10)
        Targetzone_pub=rospy.Publisher('Targetzone',Point32,queue_size=10)
        rate = rospy.Rate(30.0)
        go_flag=Bool() # run mpc_multipoints_hard_constraints
        flag=0
   
        while not rospy.is_shutdown() :
             # Create Point32 message
            point = Point32()
            point.x = 920*1.5037594e-3
            point.y = 470*1.5306122e-3
            point.z = 40*1.5037594e-3
            Targetzone_pub.publish(point)
            if Frame.image is not None and flag==0 and Robot.flag==1:
                wri = cv2.VideoWriter('record.avi', cv2.VideoWriter_fourcc(*'XVID'), 120, (1229-63,520-44), True)
                P=(int(point.x/1.5037594e-3),int(point.y /1.5306122e-3))
                # cv2.rectangle(Frame.image,(0,0),P,(255, 0, 0),3)
                cv2.rectangle(Frame.image,(P[0],0),(1229-63,520-44),(255, 0, 0),3)
                # cv2.circle(Frame.image, P,int((point.z+30)/1.5037594e-3), (255, 255, 255), 3,8,0)
                distence=1000
                Target_ID=Int8()
                Target_ID.data=0
                for i in range(len(Robot.robotID)):
                    if Robot.robotID[i]>2 and Robot.robotID[i]<10:
                        ID=Robot.robotID[i]
                        # d=(Robot.robotx[ID-1]- point.x)**2+(Robot.roboty[ID-1]-point.y)**2
                        d=(Robot.robotx[ID-1]- point.x)**2
                        if (Robot.robotx[ID-1]<point.x) and d<distence:
                            distence=d
                            Target_ID.data=ID
                TargetID_pub.publish(Target_ID.data)
                if Target_ID.data!=0:
                    go_flag.data=True
                    mpc1_flag_pub.publish(go_flag.data)
                flag=1

            if flag==1:
                
                if F.transport_flag is True and F.enclose_flag is True:
                    distence=1000
                    for i in range(len(Robot.robotID)):
                        if Robot.robotID[i]>2 and Robot.robotID[i]<10:
                            ID=Robot.robotID[i]
                            # d=(Robot.robotx[ID-1]- point.x)**2+(Robot.roboty[ID-1]-point.y)**2
                            d=(Robot.robotx[ID-1]- point.x)**2
                            if (Robot.robotx[ID-1]<point.x) and d<distence:
                                distence=d
                                Target_ID.data=int(ID)
                    TargetID_pub.publish(Target_ID.data)
                    go_flag.data=True
                    mpc1_flag_pub.publish(go_flag.data)
                else:
                    TargetID_pub.publish(Target_ID.data)
                    go_flag.data=False
                    mpc1_flag_pub.publish(go_flag.data)

                P=(int(point.x/1.5037594e-3),int(point.y /1.5306122e-3))
                cv2.rectangle(Frame.image,(P[0],0),(1229-63,520-44),(255, 0, 0),3)
                if feature.middlepoint.x!=0 and F.enclose_flag is False:
                    for xk in range(len(feature.feature_point.points)):
                        center=(int(feature.feature_point.points[xk].x),int(feature.feature_point.points[xk].y))
                        cv2.circle(Frame.image, center, 2, (0, 0, 255), -1)
                wri.write(Frame.image)
                cv2.imshow("frame",Frame.image)
                cv2.waitKey(1)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    wri.release()