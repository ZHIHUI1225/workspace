#!/usr/bin/env python -m memory_profiler
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
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
from scipy.special import comb
import matplotlib.pyplot as plt                                 # TF坐标变换库
from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from numpy import linalg as LA
import scipy.optimize as opt
N_target=3 # feature points number
class Point_tube:
    def __init__(self,name):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.sub = rospy.Subscriber(name, PoseArray, self.tube_callback,queue_size=10)
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
    def __init__(self,QRID):
        self.enclose_flag=False
        self.transport_flag=False
        self.enclose_sub=rospy.Subscriber('encloseflag'+str(QRID),Bool,self.enclose_flag_callback,queue_size=10)
        self.transport_sub=rospy.Subscriber('transportflag'+str(QRID),Bool,self.transport_flag_callback,queue_size=10)
    def enclose_flag_callback(self,msg):
        self.enclose_flag=msg.data
    def transport_flag_callback(self,msg):
        self.transport_flag=msg.data

class Bezier_polygon():
    def __init__(self):
        self.P=PointCloud()
        self.flag_side=[]
        self.sub=rospy.Subscriber('/controlpoints1',Float32MultiArray,self.sub_callback,queue_size=10)
    def sub_callback(self,msg):
        self.P=PointCloud()
        point=Point32()
        for i in range(N_target+2):
            point=Point32()
            point.x= msg.data[2*i]
            point.y= msg.data[2*i+1]
            self.P.points.append(point)

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

if __name__ == '__main__':
    try:
        rospy.init_node('record')
        Frame=frame_image()
        Robot = QRrobot()
        pointname=rospy.get_param('~feature')
        QRID=rospy.get_param('~QRID')
        F=FLAG(QRID)
        F2=FLAG(2)
        Polygon=Bezier_polygon()
        feature=Point_tube(pointname)
        feature34=Point_tube('/feature_points34')
        TargetID_pub=rospy.Publisher('TargetID'+str(QRID),Int8,queue_size=10)
        mpc1_flag_pub=rospy.Publisher('goflag'+str(QRID),Bool,queue_size=10)
        Targetzone_pub=rospy.Publisher('Targetzone'+str(QRID),Point32,queue_size=10)
        rate = rospy.Rate(30.0)
        go_flag=Bool() # run mpc_multipoints_hard_constraints
        flag=0
        R=70 # sense radiu
        while not rospy.is_shutdown() :
                     
            if Frame.image is not None and flag==0 and Robot.flag==1:
                wri = cv2.VideoWriter('record'+str(QRID)+'.avi', cv2.VideoWriter_fourcc(*'XVID'), 120, (1229-63,520-44), True)
                h,w,c=Frame.image.shape #width height channel
                 # Create Point32 message
                point = Point32()
                point.z = 45*1.5037594e-3
                if QRID==1:
                    point.x = 150*1.5037594e-3
                    point.y = 150*1.5306122e-3
                    Targetzone_pub.publish(point)
                    distence=1000
                    Target_ID=Int8()
                    Target_ID.data=0
                    for i in range(len(Robot.robotID)):
                        if Robot.robotID[i]>4 and Robot.robotID[i]<10:
                            ID=Robot.robotID[i]
                            d=(Robot.robotx[ID-1]- point.x)**2+(Robot.roboty[ID-1]-point.y)**2
                            # d=(Robot.robotx[ID-1]- point.x)**2
                            if d<distence and (Robot.robotx[ID-1]>point.x+50*1.5306122e-3 or Robot.roboty[ID-1]>point.y+30*1.5306122e-3):
                                distence=d
                                Target_ID.data=ID
                    TargetID_pub.publish(Target_ID.data)
                else:
                    point.x = (w-150)*1.5037594e-3
                    point.y = (h-150)*1.5306122e-3
                    Targetzone_pub.publish(point)
                    distence=1000
                    Target_ID=Int8()
                    Target_ID.data=0
                    for i in range(len(Robot.robotID)):
                        if Robot.robotID[i]>4 and Robot.robotID[i]<10:
                            ID=Robot.robotID[i]
                            d=(Robot.robotx[ID-1]- point.x)**2+(Robot.roboty[ID-1]-point.y)**2
                            # d=(Robot.robotx[ID-1]- point.x)**2
                            if d<distence and (Robot.robotx[ID-1]<point.x-70*1.5306122e-3 or Robot.roboty[ID-1]<point.y-50*1.5306122e-3):
                                distence=d
                                Target_ID.data=ID
                    TargetID_pub.publish(Target_ID.data)
                
                
                # P=(int(point.x/1.5037594e-3),int(point.y /1.5306122e-3))
                # cv2.rectangle(Frame.image,(0,0),P,(255, 0, 0),3)
                # cv2.rectangle(Frame.image,(P[0],0),(1229-63,520-44),(255, 0, 0),3)
                # cv2.circle(Frame.image, P,int((point.z+30)/1.5037594e-3), (255, 255, 255), 3,8,0)
               
                
                if Target_ID.data!=0:
                    go_flag.data=True
                    mpc1_flag_pub.publish(go_flag.data)
                flag=1

            if flag==1:  
                if F.transport_flag is True and F.enclose_flag is True:
                    distence=1000
                    for i in range(len(Robot.robotID)):
                        if Robot.robotID[i]>4 and Robot.robotID[i]<10: 
                            ID=Robot.robotID[i]
                            d=(Robot.robotx[ID-1]- point.x)**2+(Robot.roboty[ID-1]-point.y)**2
                            # d=(Robot.robotx[ID-1]- point.x)**2
                            if QRID==1:
                                if d<distence and (Robot.robotx[ID-1]>point.x+50*1.5306122e-3 or Robot.roboty[ID-1]>point.y+30*1.5306122e-3):
                                    distence=d
                                    Target_ID.data=int(ID)
                                    TargetID_pub.publish(Target_ID.data)
                                    go_flag.data=True
                                    mpc1_flag_pub.publish(go_flag.data)
                            else:
                                if d<distence and (Robot.robotx[ID-1]<point.x-70*1.5306122e-3 or Robot.roboty[ID-1]<point.y-50*1.5306122e-3):
                                    distence=d
                                    Target_ID.data=int(ID)
                                    TargetID_pub.publish(Target_ID.data)
                                    go_flag.data=True
                                    mpc1_flag_pub.publish(go_flag.data)
                else:
                    
                    TargetID_pub.publish(Target_ID.data)
                    go_flag.data=False
                    mpc1_flag_pub.publish(go_flag.data)
                    Targetzone_pub.publish(point)
                # if QRID==2:
                P=(int(200),int(200))
                cv2.rectangle(Frame.image,(0,0),(P[0],P[1]),(255, 0, 0),3)
                # else:
                # P=(int(w-200),int(h-200))
                # cv2.rectangle(Frame.image,(P[0],P[1]),(w,h),(255, 0, 0),3)
                if QRID==1:
                    polygon_points=[]
                    if len(Polygon.P.points)!=0 and F.enclose_flag is False:
                        for ip in range(len(Polygon.P.points)):
                            center=(int(Polygon.P.points[ip].x),int(Polygon.P.points[ip].y))
                            # cv2.circle(Frame.image, center, 3, (255, 0, 255), -1)
                            polygon_points.append([int(Polygon.P.points[ip].x),int(Polygon.P.points[ip].y)])
                        # cv2.polylines(Frame.image,[np.array(polygon_points)],isClosed=True,color=(0,255,0),thickness=2)
                        [xvals, yvals]=bezier_curve(polygon_points)
                        points = np.column_stack((xvals, yvals)).astype(np.int32)
                        # Draw the curve by connecting the points
                        color = (0, 255, 255)  # Green color in BGR format
                        is_closed = False  # Change to True if the curve should be closed
                        thickness = 2
                        cv2.polylines(Frame.image, [points], is_closed, color, thickness)
                    if feature.middlepoint.x!=0 and F.enclose_flag is False:
                        for xk in range(len(feature.feature_point.points)):
                            center=(int(feature.feature_point.points[xk].x),int(feature.feature_point.points[xk].y))
                            cv2.circle(Frame.image, center, 2, (0, 0, 255), -1)
                        
                    if feature34.middlepoint.x!=0 and F2.enclose_flag is False:
                        for xk in range(len(feature34.feature_point.points)):
                            center34=(int(feature34.feature_point.points[xk].x),int(feature34.feature_point.points[xk].y))
                            cv2.circle(Frame.image, center34, 2, (0, 0, 255), -1)
                    # center=(int(feature.feature_point.points[1].x),int(feature.feature_point.points[1].y))
                    # cv2.circle(Frame.image, center, R, (0, 255, 255), 1)
                # center=(int(Robot.robotx[0]/1.5037594e-3),int(Robot.roboty[0]/1.5306122e-3 ))
                # cv2.circle(Frame.image, center, R, (0, 255, 255), 1)
                # center=(int(Robot.robotx[1]/1.5037594e-3),int(Robot.roboty[1]/1.5306122e-3 ))
                # cv2.circle(Frame.image, center, R, (0, 255, 255), 1)
                wri.write(Frame.image)
                cv2.imshow("frame",Frame.image)
                cv2.waitKey(1)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    wri.release()