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
L_real=200* 1.5037594e-3 #the length of tube
N_target=3
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

class TUBE:
    def __init__(self):
        self.PA=PoseArray()
        self.sub = rospy.Subscriber('/line_without_QR', PointCloud, self.tube_callback,queue_size=10)

    def tube_callback(self, msg): # tube msg
        self.PA=PoseArray()      
        self.PA.header = msg.header
        self.PA.header.frame_id="world"
        # 遍历PointCloud中的每个点
        for point in msg.points:
            pose = Pose()
            # 将点的位置设置为Pose的位置
            pose.position = point
            # 将Pose添加到PoseArray中
            self.PA.poses.append(pose)
       # rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.PC)

#  red points of the tube
class Redpoints:
    def __init__(self):
        self.points=PoseArray()
        self.sub=rospy.Subscriber('/red_points',PointCloud,self.red_callback,queue_size=10)
    def red_callback(self,msg):
        self.points=PoseArray()
        self.points.header=msg.header
        self.points.header.frame_id="world"
        for point in msg.points:
            pose=Pose()
            pose.position=point
            self.points.poses.append(pose)

class Robot2:
    def __init__(self):
        self.p=PoseArray()
        self.sub=rospy.Subscriber('/robot',robot_pose_array,self.posecallback,queue_size=10)
    def posecallback(self,msg):
        self.p=PoseArray()
        self.p.header.frame_id="world"
        for i in range(len(msg.robot_pose_array)):
            ID = msg.robot_pose_array[i].ID.data
            if ID == 12:
                pose=Pose()
                pose.position.x= msg.robot_pose_array[i].position.x
                pose.position.y = msg.robot_pose_array[i].position.y
                pose.position.z = msg.robot_pose_array[i].position.z
                self.p.poses.append(pose)

class Robot1:
    def __init__(self):
        self.p=PoseArray()
        self.sub=rospy.Subscriber('/robot',robot_pose_array,self.posecallback,queue_size=10)
    def posecallback(self,msg):
        self.p=PoseArray()
        self.p.header.frame_id="world"
        for i in range(len(msg.robot_pose_array)):
            ID = msg.robot_pose_array[i].ID.data
            if ID == 11:
                pose=Pose()
                pose.position.x= msg.robot_pose_array[i].position.x
                pose.position.y = msg.robot_pose_array[i].position.y
                pose.position.z = msg.robot_pose_array[i].position.z
                self.p.poses.append(pose)


class frame_image():
    def __init__(self):
        # Params
        self.image=None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(30)
        # Subscribers
        rospy.Subscriber('/camera/image',Image,self.image_callback,queue_size=10)

    def image_callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        # cv2.imshow("image_tf",self.image)
        # cv2.waitKey(3)

class TFListener:

    def __init__(self,source_frame,target_frame):
        # rospy.init_node('tf_listener')
        self.source_frame =source_frame
        self.target_frame = target_frame
        self.tf_listener = TransformListener()
        self.tf_buffer = TransformerROS()
        # self.timer = rospy.Timer(rospy.Duration(1.0), self.on_timer)

    def on_timer(self, event):
        try:
            now = rospy.Time.now()
            # trans = self.tf_buffer.lookupTransform(
            #     self.target_frame,
            #     self.source_frame,
            #     now)
            (tran, rot) = self.tf_listener.lookupTransform(self.target_frame, self.source_frame , rospy.Time(0))
            print(tran)
            print("success")
        except Exception as ex:
            rospy.loginfo(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return
          
    def transform(self,input):
        #from source to target (posestamped)
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(self.target_frame,self.source_frame,now,rospy.Duration(1.0))
            (tran, rot) = self.tf_listener.lookupTransform(self.target_frame, self.source_frame , rospy.Time(0))
            transformed_pose = PoseArray()
            transformed_pose.header = self.target_frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = input.header.frame_id
            for pose in input.poses:
                pose_stamped.pose = pose
                transformed_pose_stamped = self.tf_listener.transformPose(self.target_frame, pose_stamped)
                transformed_pose.poses.append(transformed_pose_stamped.pose)
            return transformed_pose
        except Exception as ex:
            rospy.loginfo(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return
    def transform_back(self,input):
        # from target to source
        # input list of PoseStamped()
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(self.source_frame,self.target_frame,now,rospy.Duration(1.0))
            (tran, rot) = self.tf_listener.lookupTransform(self.source_frame, self.target_frame , rospy.Time(0))
            transformed_pose = PoseArray()
            transformed_pose.header.frame_id = self.source_frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.target_frame
            for i in range(len(input)):
                pose_stamped.pose.position=input[i].pose.position
                transformed_pose_stamped = self.tf_listener.transformPose(self.source_frame, pose_stamped)
                transformed_pose.poses.append(transformed_pose_stamped.pose)
            return transformed_pose
        except Exception as ex:
            rospy.loginfo(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return
# def pointcloud_to_posearray(pc):
#     # 创建一个空的PoseArray对象
#     pose_array = PoseArray()
#     # 设置PoseArray的header
#     pose_array.header = pc.header
#     # 创建一个新的Pose对象
#     pose = Pose()
#     # 遍历PointCloud中的每个点
#     for point in pc.points:
#         # 将点的位置设置为Pose的位置
#         pose.position = point
#         # 将Pose添加到PoseArray中
#         pose_array.poses.append(pose)
#     return pose_array

# def polynomial_curve_fit(key_point, n):
#     # Number of key points
#     N = len(key_point)
#     #print(key_point)
#     # Construct matrix X
#     X = np.zeros((n + 1, n + 1))
#     for i in range(n + 1):
#         for j in range(n + 1):
#             for k in range(N):
#                 X[i][j] += key_point[k][0] ** (i + j)
#     # Construct matrix Y
#     Y = np.zeros((n + 1, 1))
#     for i in range(n + 1):
#         for k in range(N):
#             Y[i][0] += key_point[k][0] ** i * key_point[k][1]
#     # Solve matrix A
#     A = np.zeros((n + 1, 1))
#     A = np.linalg.solve(X, Y)
#     return A
def convert2posearray(input):
    transformed_pose = PoseArray()
    transformed_pose.header.frame_id = "local"
    for i in range(len(input)):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "local"
        pose_stamped.pose.position=input[i].position
        transformed_pose.poses.append(pose_stamped.pose)
    return transformed_pose
            
def my_func(x):
    if x < 0:
        return 0
    else:
        return x
        
class tubeshape():
    def __init__(self,length:float,p1:np.array,p2:np.array):
            self.l=length # length of tube
            # position of two ends
            self.p1=p1 
            self.p2=p2
            #distance of two ends
            self.x0=LA.norm(self.p1-self.p2,2)

    def update(self,p1,p2):
        self.p1=p1
        self.p2=p2
        self.x0=LA.norm(self.p1-self.p2,2)
# from local coordinate to world cooridinate
    def transformR(self):
        theta=math.atan2(self.p2[1]-self.p1[1],self.p2[0]-self.p1[0])
        R=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        return R
    def get_a(self):
        def lengthl(delta):
            k=1/np.sqrt(1+delta)
            F=(k**4-28*k**2+64)/(4*k**4-40*k**2+64)-self.l*np.pi/(2*self.x0)/np.sqrt(1+1/delta)
            return F
        delta0=4*self.x0**2/np.pi**2/(self.l**2-self.x0**2)
        delta,_,ier,_ =opt.fsolve(lengthl,my_func(delta0),xtol=1e-05,full_output=True)
        if ier==1:
            a=-np.sqrt(self.x0**2/(np.pi**2*delta))
        else:
            a=0
        return a
    
    def getshape(self):
        x=np.arange(0,self.x0,self.x0/100)
        a=self.get_a()
        if a==0:
            P=[]
        else:
            y=a*np.sin(np.pi*x/self.x0)
            R=self.transformR()
            P=np.dot(R,np.array([x,y]))
            P[0,:]=np.transpose(np.array(P[0,:]+self.p1[0]))
            P[1,:]=np.transpose(np.array(P[1,:]+self.p1[1]))
        return P
    def get_points(self,n):
        points=[]
        y=self.getshape()
        if y!=[]:
            for i in range(1,n+1):
            #for i in range(int(len(y[0,:])/(n+1)),len(y[0,:])-1,int(len(y[0,:])/(n+1))):
                points.append(y[:,i*int(len(y[0,:])/(n+1))])
        return points
    def get_control_points(self,degree):
        P=self.getshape()
        points = []
        xpoints=P[0]
        ypoints=P[1]
        for i in range(len(xpoints)):
            points.append([xpoints[i],ypoints[i]])
        # Get the Bezier parameters based on a degree.
        data = get_bezier_parameters(xpoints, ypoints, degree=4)
        x_val = [x[0] for x in data]
        y_val = [x[1] for x in data]
        return data

if __name__ == '__main__':
    try:
        rospy.init_node('tf_listener_node')
        tube=TUBE()
        RedPoints=Redpoints()
        tf_w2l= TFListener('world','local')
        Frame=frame_image()
        robot2=Robot2()
        robot1=Robot1()
        # feature_points_pub=rospy.Publisher('feature_points',PoseArray,queue_size=1)
        redpoints_pub=rospy.Publisher('feature_points',PoseArray,queue_size=1)
        # listener_w2l = tf.TransformListener()
        rate = rospy.Rate(50.0)
        flag=0
        while not rospy.is_shutdown() :
            if Frame.image is not None and flag==0:
                wri = cv2.VideoWriter('zhihui.avi', cv2.VideoWriter_fourcc(*'XVID'), 10, (1229-63,520-44), True)
                flag=1
            if  RedPoints.points.poses and Frame.image is not None and robot1.p.poses and robot2.p.poses:
                robot2_local=tf_w2l.transform(robot2.p)
                robot1_local=tf_w2l.transform(robot1.p)
                line_local=tf_w2l.transform(RedPoints.points)
                if line_local:
                    # print(line_local.poses)
                    sorted_line = sorted(line_local.poses, key=lambda pose: pose.position.x)
                    #calculate length
                    L=0
                    k=0
                    xlist=[]
                    ylist=[]
                    redlist=[]
                    nu=0
                    for i in range(1, len(sorted_line)):
                        l = math.hypot(sorted_line[k].position.x - sorted_line[i].position.x, sorted_line[k].position.y - sorted_line[i].position.y)
                        if l < 20:
                            xlist.append(sorted_line[i].position.x)
                            ylist.append(sorted_line[i].position.y)           
                        else:
                            if len(xlist)!=0:
                                pose_stamped = PoseStamped()
                                pose_stamped.header.frame_id = 'local'
                                pose_stamped.pose.position.x=sum(xlist)/len(xlist)
                                pose_stamped.pose.position.y=sum(ylist)/len(ylist)
                                redlist.append(pose_stamped)
                                nu=nu+1
                            xlist.clear()
                            ylist.clear()
                        if nu==3:
                            break
                        k = i
                    if nu==2 and len(xlist)!=0:
                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = 'local'
                        pose_stamped.pose.position.x=sum(xlist)/len(xlist)
                        pose_stamped.pose.position.y=sum(ylist)/len(ylist)
                        redlist.append(pose_stamped)
                    red_points=tf_w2l.transform_back(redlist)
                    redpoints_pub.publish(red_points)
                    # for pose in feature_points_local.poses:
                    L_real=200* 1.5037594e-3#the length of tube
                    r1=np.array([400* 1.5037594e-3,300* 1.5306122e-3 ])
                    r2=np.array([550* 1.5037594e-3,300* 1.5306122e-3 ])
                    Tube=tubeshape(length=L_real,p1=r1,p2=r2)
                    xt=np.array(Tube.get_points(N_target)).reshape(-1,1)
                    center=(int(r1[0]/1.5037594e-3),int(r1[1]/1.5306122e-3))
                    cv2.circle(Frame.image, center, 3, (255, 0, 255), -1)
                    center=(int(r2[0]/1.5037594e-3),int(r2[1]/1.5306122e-3))
                    cv2.circle(Frame.image, center, 3, (255, 0, 255), -1)
                    for i in range(0,len(xt),2):
                        center=(int(xt[i]/1.5037594e-3),int(xt[i+1]/1.5306122e-3))
                        cv2.circle(Frame.image, center, 3, (255, 0, 50*i), -1)
                    if red_points is not None:
                        for pose in red_points.poses:
                            center=(int(pose.position.x),int(pose.position.y))
                            cv2.circle(Frame.image, center, 2, (0, 0, 255), -1)
                    wri.write(Frame.image)
                    cv2.imshow("frame",Frame.image)
                    cv2.waitKey(1)
                    
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    wri.release()