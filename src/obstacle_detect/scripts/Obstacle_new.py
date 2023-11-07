from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
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
import casadi as ca
import casadi.tools as ca_tools
import time
from numpy import linalg as LA
import csv
# import env
# from draw import Draw_MPC_two_agents_withtube
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
#import Staticcontrol
#import LOScontrol
import time
from scipy.optimize import fsolve
from scipy.special import ellipe
from cv_bridge import CvBridge
import scipy.optimize as opt
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
import pandas as pd
# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate

N_target=3 # feature points number
plt.ion()

class Obstacle_QR:
    def __init__(self):
        self.points=np.zeros((4,2))
        self.flag=0
        self.senseP=np.zeros((2,2))
        self.flag_in=True
        self.flag_side=1 #True
        self.vector=[]
        self.sub = rospy.Subscriber('/obstacle', robot_pose_array, self.pose_callback,queue_size=10)
    def pose_callback(self, msg): # feedback means actual value.
        for i in range(len(msg.robot_pose_array)):
            ID=msg.robot_pose_array[i].ID.data
            self.points[int(ID-2),0]=msg.robot_pose_array[i].position.x * 1.5037594e-3
            self.points[int(ID-2),1]=msg.robot_pose_array[i].position.y * 1.5306122e-3 
        self.flag=1
        self.vector=[]
        self.vector.append(self.points[1]-self.points[0])
        self.vector.append(self.points[2]-self.points[3])
        self.vector.append(self.points[3]-self.points[0])
        self.vector.append(self.points[2]-self.points[1])
    def calculateP(self,P):
        # weather between vector 0 and 1
        if np.cross(self.vector[0],P-self.points[0])*np.cross(self.vector[1],P-self.points[3])<=0:
            if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
                self.flag_in=True
            elif np.cross(self.vector[2],P-self.points[0])>0:
                self.flag_in=False
                self.senseP[0,:]=self.points[0]
                self.senseP[1,:]=self.points[3]
                self.flag_side=1
            else:
                self.flag_in=False
                self.senseP[0,:]=self.points[1]
                self.senseP[1,:]=self.points[2]
                self.flag_side=1
        elif np.cross(self.vector[0],P-self.points[0])<0:
            self.flag_in=False
            if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
                self.senseP[0,:]=self.points[0]
                self.senseP[1,:]=self.points[1]
                self.flag_side=1
            elif np.cross(self.vector[2],P-self.points[0])>0:
                self.senseP[0,:]=self.points[3]
                self.senseP[1,:]=self.points[1]
                self.flag_side=0
            else:
                self.senseP[0,:]=self.points[2]
                self.senseP[1,:]=self.points[0]
                self.flag_side=0
        else:
            self.flag_in=False
            if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
                self.senseP[0,:]=self.points[3]
                self.senseP[1,:]=self.points[2]
                self.flag_side=1
            elif np.cross(self.vector[2],P-self.points[0])>0:
                self.senseP[0,:]=self.points[0]
                self.senseP[1,:]=self.points[2]
                self.flag_side=0
            else:
                self.senseP[0,:]=self.points[3]
                self.senseP[1,:]=self.points[1]
                self.flag_side=0
        return self.senseP

class Point_tube:
    def __init__(self,pointname):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.sub = rospy.Subscriber(pointname, PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        self.feature_point=PointCloud()
        for pose in msg.poses:
            pose.position.x= pose.position.x* 1.5037594e-3
            pose.position.y= pose.position.y* 1.5306122e-3 
            self.feature_point.points.append(pose.position)
        if len(msg.poses)%2==1: # odd
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses)+1)/2)].x* 1.5037594e-3 #to m
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses)+1)/2)].y* 1.5306122e-3 
        else:
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses))/2)].x* 1.5037594e-3
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses))/2)].y* 1.5306122e-3 
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
            self.robotID[int(ID-1)]=msg.robot_pose_array[i].ID.data
            self.robotyaw[int(ID-1)]=msg.robot_pose_array[i].yaw
        self.flag=1

class Targe_ID:
    def __init__(self,QRID):
        self.flag=False
        self.transport_flag=False
        self.ID=0
        self.sub=rospy.Subscriber('goflag'+str(QRID),Bool,self.flag_callback,queue_size=10)
        self.subID=rospy.Subscriber('TargetID'+str(QRID),Int8,self.ID_callback,queue_size=10)
        self.transport_sub=rospy.Subscriber('transportflag'+str(QRID),Bool,self.transport_flag_callback,queue_size=10)
    def flag_callback(self,msg):
        self.flag=msg.data
    def transport_flag_callback(self,msg):
        self.transport_flag=msg.data
    def ID_callback(self,msg):
        self.ID=int(msg.data)

class Plotupdate():
    def __init__(self,name,yname,num,Label):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.figure.set_figwidth(4) 
        self.figure.set_figheight(2) 
        self.line_objects = []
        self.num=num
        if num==1:
            line, = self.ax.plot([], [])
            self.line_objects.append(line)
        else:
            for i in range(num):
                line, = self.ax.plot([], [], label=Label[i])
                self.line_objects.append(line)
        self.starttime= time.time()
        self.xdata=[]
        self.ax.set_autoscaley_on(True)
        self.ax.grid()
        self.figure.suptitle(name)
        self.ax.legend(loc='upper left')
        self.ax.set_xlabel('time(s)',fontsize=8)
        self.ax.set_ylabel(yname,fontsize=8)
        self.figure.suptitle(name,fontsize=8)
        self.ax.legend(loc='upper left')
        self.ax.tick_params(axis='x', labelsize=8)
        self.ax.tick_params(axis='y', labelsize=8)
    def on_running(self, error):
        timec = time.time()
        self.xdata.append(timec-self.starttime)
        for i in range(self.num):
            self.line_objects[i].set_xdata(self.xdata)
            self.line_objects[i].set_ydata(error[i])
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
    def save_plot(self,figname,dataname):
        self.figure.savefig(figname)
        fp = open(dataname, mode='a+', newline='')
        dp = csv.writer(fp)
        for i in range(self.num):
            dp.writerow(self.line_objects[i].get_xdata())
            dp.writerow(self.line_objects[i].get_ydata())   
        fp.close() 

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


def getdcos_to_x(X1,X2,qm,pt,J1,J2):
    l1=np.linalg.norm(pt-qm)
    l2=np.linalg.norm((X1+X2)/2-qm)
    z1=(pt-qm)/l1
    z2=((X1+X2)/2-qm)/l2
    I=np.eye((2))
    M1=np.matmul(I- np.matmul(z1,np.transpose(z1))/l1,-J1)
    M2=np.matmul(I- np.matmul(z2,np.transpose(z2))/l2,(I/2))
    D=np.matmul(np.transpose(z1),M2)
    return D
def getdcos_to_qm(X1,X2,qm,pt,J1,J2):
    l1=np.linalg.norm(pt-qm)
    l2=np.linalg.norm((X1+X2)/2-qm)
    z1=(pt-qm)/l1
    z2=((X1+X2)/2-qm)/l2
    I=np.eye((2))
    M1=np.matmul(I- np.matmul(z1,np.transpose(z1))/l1,(I))
    M2=np.matmul(I- np.matmul(z2,np.transpose(z2))/l2,(I))
    D=-np.matmul(np.transpose(z2),M1)+np.matmul(np.transpose(z1),M2)
    return D

def distence_to_x(pv,pt,Qr):
    D=np.matmul(np.transpose(pv-pt),(Qr+np.transpose(Qr)))/(N_target+2)
    return D

def get_derivative(x,P,a,b):
    # update the cost function                   
    x1=np.concatenate(x)
    Ctheta=np.dot((P[0] - x1[:2]), (P[1]- x1[:2])) / np.linalg.norm(P[0]- x1[:2]) / np.linalg.norm(P[1]-x1[:2])
    Ctanh=math.tanh(a*Ctheta+b)
    l1=np.linalg.norm(P[0] - x1[:2])
    l2=np.linalg.norm(P[1]- x1[:2])
    z1=(P[0] - x1[:2])/ l1
    z2=(P[1]- x1[:2]) / l2
    DCtheta_vector=(1/l2-Ctanh/l1)*z1+(1/l1-Ctanh/l2)*z2
    DCtanh=a*(1-np.square(math.tanh(2.5)))*DCtheta_vector
    return DCtanh
    
def get_cos_derivative(x,P):
    # update the cost function                   
    x1=np.concatenate(x)
    Ctheta=np.dot((P[0] - x1[:2]), (P[1]- x1[:2])) / np.linalg.norm(P[0]- x1[:2]) / np.linalg.norm(P[1]-x1[:2])
    l1=np.linalg.norm(P[0] - x1[:2])
    l2=np.linalg.norm(P[1]- x1[:2])
    z1=(P[0] - x1[:2])/ l1
    z2=(P[1]- x1[:2]) / l2
    DCtheta_vector=(1/l2-Ctheta/l1)*z1+(1/l1-Ctheta/l2)*z2
    return DCtheta_vector

if __name__ == '__main__':
    try:
        rospy.init_node('Obstacle_avoidence')
        pointname=rospy.get_param('~feature')
        QRID=rospy.get_param('~QRID')
        # ForceK_pub=rospy.Publisher('ForceK'+str(QRID),Float32MultiArray,queue_size=10)
        intersectionP_pub=rospy.Publisher('InterP'+str(QRID),Float32MultiArray,queue_size=10)
        IP_flag_pub=rospy.Publisher('Flag_side'+str(QRID),Float32MultiArray,queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        feature=Point_tube(pointname)
        Targe_id=Targe_ID(QRID)
        
        F=FLAG(QRID)
        r_object=32
        rate = rospy.Rate(30)
        # Errorplot=PlotUpdate2(1)
        object_flag=0
        Obstacle=Obstacle_QR()
        force1_x=[]
        force1_cos=[]
        force1_ao=[]
        force2_x=[]
        force2_cos=[]
        force2_ao=[]
        force_x_tube=[]
        force_cos_tube=[]
        force_ao_tube=[]
        J=np.array([[ 0.28348009 ,-0.10893156, -0.0462114 , -0.06149945],\
            [ 0.15820577  ,0.3135021 , -0.23376026 , 0.1723733 ],\
            [ 0.13487716 ,-0.07193869 , 0.16223159 ,-0.02951396],\
            [ 0.2540872  , 0.27042158 ,-0.35836604 , 0.2319037 ],\
            [ 0.00593047  ,0.06580107 , 0.30361478 ,-0.03436256],\
            [ 0.17973368  ,0.22378411 ,-0.3361492  , 0.14395073]])
        while not rospy.is_shutdown():
             if Robot.flag==1 and feature.middlepoint.x!=0 and Targe_id.transport_flag is False and F.enclose_flag is False and Obstacle.flag==1 :
                if object_flag==0 and Targe_id.ID!=0 :
                    IDi=Targe_id.ID-1
                    Target_circle=np.array([Robot.robotx[IDi], Robot.roboty[IDi], r_object* 1.5037594e-3])
                    Forceplot1=Plotupdate('Force1','derivative',3,['attrative','steer','push'])
                    Forceplot2=Plotupdate('Force2','derivative',3,['attrative','steer','push '])
                    Forceplot_tube=Plotupdate('Force_tube','derivative',3,['attrative ','steer','push '])
                    # Ko_tube=np.zeros((N_target,1))
                    # Ko=np.zeros((2,1))
                    # flag_obstacle=np.zeros((N_target+2,1))
                    xs=[]
                    # for i in range(N_target+2):
                    xs.append(Target_circle[:2])
                    xs=np.array(xs).reshape(-1,1)
                    object_flag=1
                if object_flag==1:
                    if len(feature.feature_point.points)!=3:
                        x0[:6]=np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]).reshape(-1,1)
                    else:
                        x0= [Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]
                        for xk in range(N_target):
                            x0.append(feature.feature_point.points[xk].x)
                            x0.append(feature.feature_point.points[xk].y)
                    x0=np.array(x0).reshape(-1,1)
                    x1=np.concatenate(x0)
                    x_center=x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12]
                    x_center=(x_center/(N_target+2)).reshape(1,-1)

                    intersectionP=[]
                    Flag_side=[]
                    intersectionP.append(Obstacle.calculateP(x1[:2]).copy())
                    Flag_side.append((Obstacle.flag_side))
                    intersectionP.append(Obstacle.calculateP(x1[3:5]).copy())
                    Flag_side.append((Obstacle.flag_side))
                    intersectionP.append(Obstacle.calculateP(x1[6:8]).copy())
                    Flag_side.append((Obstacle.flag_side))
                    intersectionP.append(Obstacle.calculateP(x1[8:10]).copy())
                    Flag_side.append((Obstacle.flag_side))
                    intersectionP.append(Obstacle.calculateP(x1[10:12]).copy())
                    Flag_side.append((Obstacle.flag_side))
                    inter=np.zeros((4*(N_target+2),1))
                    for i in range(5):
                        inter[4*i:4*(i+1),0]=np.concatenate(intersectionP[i])
                    iP=Float32MultiArray(data=inter)
                    intersectionP_pub.publish(iP)
                    IP_flag_pub.publish(data=Flag_side)
                    #the middle point of the tube
                    Dtanh_tube=[]
                    Qr=1*np.eye(2)
                    ktheta=0.03
                    KO=0.1
                    Dx=distence_to_x((x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12])/(2+N_target),xs,Qr)
                    Dcos=getdcos_to_qm(x0[:2],x0[3:5],x0[8:10],xs,J[2:4,:2],J[2:4,-2:])
                    DCtanh=0
                    for f_tube in range(N_target):
                        if f_tube==1:
                            Ctheta=np.dot((intersectionP[2+f_tube][0] - x1[6+2*f_tube:8+2*f_tube]), (intersectionP[2+f_tube][1]- x1[6+2*f_tube:8+2*f_tube])) / np.linalg.norm(intersectionP[2+f_tube][0]- x1[6+2*f_tube:8+2*f_tube]) / np.linalg.norm(intersectionP[2+f_tube][1]-x1[6+2*f_tube:8+2*f_tube])
                            DC=get_cos_derivative(x0[6+2*f_tube:8+2*f_tube],intersectionP[2+f_tube])
                            # if Flag_side[2+f_tube]==1:
                            #     if Ctheta >-0.5:
                            #         Dcos=0*DC
                            #     else:
                            #         Dcos=(3-12*Ctheta**2)*DC
                            # else:
                            #     if Ctheta>0.5:
                            #         Dcos=0*DC
                            #     else:
                            #         Dcos=4*Ctheta*(16*Ctheta**4-16*Ctheta**2+1)+(2*Ctheta**2-1)*(16*4*Ctheta**3-32*Ctheta)*DC
                            if Ctheta >0.5:
                                DC=0*DC
                            elif Ctheta<0:
                                DC=0*DC
                            else:
                               DC=4*Ctheta*(16*Ctheta**4-16*Ctheta**2+1)+(2*Ctheta**2-1)*(16*4*Ctheta**3-32*Ctheta)*DC
                            force_x_tube.append(np.linalg.norm(Dx))
                            force_cos_tube.append(ktheta*np.linalg.norm(Dcos))
                            force_ao_tube.append(KO*np.linalg.norm(DC))
                            Force_tube=[force_x_tube,force_cos_tube,force_ao_tube]
                            Forceplot_tube.on_running(Force_tube)

                    Dcos1=getdcos_to_x(x0[:2],x0[3:5],x0[8:10],xs,J[2:4,:2],J[2:4,-2:])
                    
                    DCosx=[np.zeros((2,1)),np.zeros((2,1))]
                    for io in range(2): 
                        Ctheta=np.dot((intersectionP[io][0] - x1[3*io:3*io+2]), (intersectionP[io][1]- x1[3*io:3*io+2])) / np.linalg.norm(intersectionP[io][0]- x1[3*io:3*io+2]) / np.linalg.norm(intersectionP[io][1]-x1[3*io:3*io+2])
                        DCos=get_cos_derivative(x0[3*io:3*io+2],intersectionP[io])
                        # if Flag_side[io]==1:
                        #     if Ctheta >-0.5:
                        #         DCosx[io]=0*DCos
                        #     else:
                        #         DCosx[io]=(3-12*Ctheta**2)*DCos
                        # else:
                        #     if Ctheta>0.5:
                        #         DCosx[io]=0*DCos
                        #     else:
                        #         DCosx[io]=4*Ctheta*(16*Ctheta**4-16*Ctheta**2+1)+(2*Ctheta**2-1)*(16*4*Ctheta**3-32*Ctheta)*DCos
                        if Ctheta >0.5:
                            DCosx[io]=0*DCos
                        elif Ctheta<0:
                            DCosx[io]=0*DCos
                        else:
                            DCosx[io]=4*Ctheta*(16*Ctheta**4-16*Ctheta**2+1)+(2*Ctheta**2-1)*(16*4*Ctheta**3-32*Ctheta)*DCos
                    #     else:
                    #         if Ctheta>math.sqrt(3)/2:
                    #             DCosx[io]=0*DCos
                    #         elif Ctheta<0.5:
                    #             DCosx[io]=0*DCos
                    #         else:
                    #             DCosx[io]=-4*Ctheta*(16*Ctheta**4-16*Ctheta**2+1)+(2*Ctheta**2-1)*(16*4*Ctheta**3-32*Ctheta)*DCos
                    force1_x.append(np.linalg.norm(Dx))
                    force1_cos.append(ktheta*np.linalg.norm(Dcos1))
                    force1_ao.append(KO*np.linalg.norm(DCosx[0]))
                    Force1=[force1_x,force1_cos,force1_ao]
                    Forceplot1.on_running(Force1)
                    force2_x.append(np.linalg.norm(Dx))
                    force2_cos.append(ktheta*np.linalg.norm(Dcos1))
                    force2_ao.append(KO*np.linalg.norm(DCosx[1]))
                    Force2=[force2_x,force2_cos,force2_ao]
                    Forceplot2.on_running(Force2)
                    if np.linalg.norm(x_center[0]-Target_circle[:2])<r_object* 1.5037594e-3*0.5:
                        # save data
                        Forceplot1.save_plot('Force1'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Force1'+str(Targe_id.ID)+'.csv')
                        Forceplot2.save_plot('Force2'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Force2'+str(Targe_id.ID)+'.csv')
                        Forceplot_tube.save_plot('Force_tube'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Force_tube'+str(Targe_id.ID)+'.csv')


    except rospy.ROSInterruptException:
        pass
