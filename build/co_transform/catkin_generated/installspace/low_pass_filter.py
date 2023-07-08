
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
import csv
import tf
import time
import matplotlib.pyplot as plt                                 # TF坐标变换库
from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from numpy import linalg as LA
import scipy.optimize as opt
from scipy.signal import butter, lfilter, freqz
from scipy.ndimage.interpolation import shift
N_target=5
class Filter:
    def __init__(self,size,channel):
        self.size=size
        self.channel=channel
        # self.ganma=ganma
        self.filterdata=np.zeros(size*channel).reshape(channel,size)#新的数据排在最前面
        # self.ganma=[np.power(ganma,i) for i in range(size)]
 
    def Clear(self):
        self.filterdata=np.zeros(self.size*self.channel).reshape(self.channel,self.size)
 
    def Add(self,data):
        #data=[x,y,z,..]
        if self.filterdata[0][0]!=0:
            for i in range(self.channel):
                if abs(data[i]-self.filterdata[i,0])<2:
                    data[i]=self.filterdata[i,0]
        self.filterdata = np.roll(self.filterdata, 1, axis=1)#移位
        self.filterdata[:,0]=0#清零
        self.filterdata[:,0]=self.filterdata[:,0]+data#加入新值
        None
    def countZeros(self,nums):
        count=0
        for i in range(len(nums)):
            if nums[i]==0:
                count+=1
        return count
    def GetResult(self):
        mean=np.zeros(self.channel)
        for i in range(self.channel):
            count=self.countZeros(self.filterdata[i,:])
            mean[i]=sum(self.filterdata[i,:])/(self.size-count)
        # result = np.dot(self.filterdata,self.ganma)
        # print("GetResult=",result)
        return mean#返回array[x,y,z..]

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
        self.controlpoints=np.zeros((1,N_target*2+4))
        self.flag=0
        # self.sub = rospy.Subscriber('/line_without_QR', PointCloud, self.tube_callback,queue_size=10)
        self.sub = rospy.Subscriber('feature_points',PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        i=0
        for pose in msg.poses:
            self.controlpoints[0][i]=pose.position.x
            self.controlpoints[0][i+1]=pose.position.y
            i=i+2
        self.flag=1


def butter_lowpass(cutoff, fs, order):
    return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y



if  __name__ == '__main__':
    try:
        rospy.init_node('low_pass_filter')
        feature=Point_tube()
        Frame=frame_image()
        F=Filter(size=3,channel=N_target*2)
        # record_points_pub=rospy.Publisher('sample_points', Point32,queue_size=1)
        filter_points_pub=rospy.Publisher('filter_points',PoseArray,queue_size=1)
        Image_points_pub=rospy.Publisher('Image_points',Image,queue_size=1)
        # Filter requirements.
        order = 4
        fs = 30.0       # sample rate, Hz
        cutoff = 3.667  # desired cutoff frequency of the filter, Hz
        # # Get the filter coefficients so we can check its frequency response.
        # b, a = butter_lowpass(cutoff, fs, order)
        rate = rospy.Rate(fs)
        # # Define the filename for the output CSV file
        # filename = 'data.csv'
        # # Open the output file in write mode with newline='' to ensure consistent line endings
        # with open(filename, 'w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csvwriter = csv.writer(csvfile)
        # data=[]
        # flag=0
        wri = cv2.VideoWriter('zhihui.avi', cv2.VideoWriter_fourcc(*'XVID'), 60, (1062, 476))
        while not rospy.is_shutdown():
            # print("time",time.time())
            if feature.flag==1 and Frame.image is not None:
                xt=feature.controlpoints[0]
                xt=np.reshape(xt,(len(xt),1))
                xt=np.hstack((xt[2:-2]))
                points=xt
                F.Add(xt)
                result=F.GetResult()
                Cpoints=PoseArray()
                for i in range(N_target):
                    pose = Pose()
                    pose.position.x=result[2*i]
                    pose.position.y=result[2*i+1]
                    Cpoints.poses.append(pose)
                filter_points_pub.publish(Cpoints)
                xs = [440, 200,420,205,400, 210,380,205,360, 200 ]
                for i in range(N_target):
                    center=(int(result[2*i]),int(result[2*i+1]))
                    cv2.circle(Frame.image, center, 1, (0, 80*i, 255), -1)
                    center=(int(xs[2*i]),int(xs[2*i+1]))
                    cv2.circle(Frame.image, center, 2, (100*i, 255, 100*i), -1)
                # bridge = CvBridge()
                # image_msg = bridge.cv2_to_imgmsg(Frame.image, encoding="bgr8")
                # Image_points_pub.publish(image_msg)
                wri.write(Frame.image)
                cv2.imshow("image",Frame.image)
                cv2.waitKey(1)
                # cv2.waitKey(1)
                # # Create a Point message and publish it to the topic
                # point_msg = Point32(x=points[2], y=points[3], z=0.0)
                # record_points_pub.publish(point_msg)
               
                
        rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
    wri.release()