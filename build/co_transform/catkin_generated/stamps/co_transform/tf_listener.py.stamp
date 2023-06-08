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

class frame_image():
    def __init__(self):
        # Params
        self.image=None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
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
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(self.source_frame,self.target_frame,now,rospy.Duration(1.0))
            (tran, rot) = self.tf_listener.lookupTransform(self.source_frame, self.target_frame , rospy.Time(0))
            transformed_pose = PoseArray()
            transformed_pose.header.frame_id = self.source_frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.target_frame
            for i in range(len(input)):
                pose_stamped.pose.position=input[i].position
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


if __name__ == '__main__':
    try:
        rospy.init_node('tf_listener_node')
        tube=TUBE()
        tf_w2l= TFListener('world','local')
        Frame=frame_image()
        feature_points_pub=rospy.Publisher('feature_points',PoseArray,queue_size=1)
        # listener_w2l = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() :
            if tube.PA.poses:
                line_local=tf_w2l.transform(tube.PA)
                if line_local:
                    # print(line_local.poses)
                    sorted_line = sorted(line_local.poses, key=lambda pose: pose.position.x)
                    #calculate length
                    L=0
                    k=0
                    for i in range(1, len(sorted_line)):
                        l = math.hypot(sorted_line[k].position.x - sorted_line[i].position.x, sorted_line[k].position.y - sorted_line[i].position.y)
                        if l > 0.5:
                            L=L+l
                            k = i

                    k = 0
                    pc_10 = []
                    delta_l=0
                    pc_10.append(sorted_line[0])
                    for i in range(1, len(sorted_line)):
                        l = math.hypot(sorted_line[k].position.x - sorted_line[i].position.x, sorted_line[k].position.y - sorted_line[i].position.y)
                        if l > 1:
                            delta_l=delta_l+l
                            k = i
                            if delta_l>L/2-1:
                                # ipt = (sorted_line[i].position.x, sorted_line[i].position.y)
                                pc_10.append(sorted_line[i])
                                delta_l=0
                                break
                    pc_10.append(sorted_line[int(len(sorted_line)-1)])        
                            # center = (int(ipt[0]), int(ipt[1]))
                           
                    # print(pc_10)
                    feature_points=tf_w2l.transform_back(pc_10)
                    feature_points_pub.publish(feature_points)
                    for pose in feature_points.poses:
                        center=(int(pose.position.x),int(pose.position.y))
                        if Frame.image is not None:
                            cv2.circle(Frame.image, center, 2, (0, 0, 255), -1)
                            xs = np.array([990, 300]).reshape(-1, 1) # final state
                            center=(int(xs[0]),int(xs[1]))
                            cv2.circle(Frame.image, center, 5, (255, 0, 0), -1)
                    if Frame.image is not None:
                        cv2.imshow("frame",Frame.image)
                        cv2.waitKey(1)
                    #曲线拟合
                    # Num=3
                    # A = np.zeros((Num + 1, 1))
                    # line = np.array([(pose.position.x, pose.position.y, pose.position.z) for pose in line_local.poses])
                    # #print(line)
                    # A = polynomial_curve_fit(line, Num)
                    # #print(A)
                    # X = [pose.position.x for pose in line_local.poses]
                    # Y = [pose.position.y for pose in line_local.poses]
                    # min_x = min(X)
                    # max_x = max(X)
                    # # Compute the fitted points
                    # points_fitted = []
                    # img = np.zeros((512,512,3),np.uint8)
                    # number=10
                    # for i in range(number):
                    #     x = i * (max_x - min_x) / (number-1) + min_x
                    #     y = 0
                    #     for j in range(Num + 1):
                    #         y += A[j][0] * x ** j  # Compute the value of the fitted function
                    #     points_fitted.append([x, y])
                    #     center = (int(round(points_fitted[i][0])), int(round(points_fitted[i][1])))
                    
        
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
