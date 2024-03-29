#!/usr/bin/env python3
# use the Aruco code  detect the tube feature points
# use cos piexewise function to avoid obstacles
from std_msgs.msg import Int8
from std_msgs.msg import Bool
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
from std_msgs.msg import Float32MultiArray
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

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=3 # feature points number
plt.ion()

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

# class Obstacle_QR:
#     def __init__(self):
#         self.points=np.zeros((4,2))
#         self.flag=0
#         self.senseP=np.zeros((2,2))
#         self.flag_in=True
#         self.vector=[]
#         self.sub = rospy.Subscriber('/obstacle', robot_pose_array, self.pose_callback,queue_size=10)
#     def pose_callback(self, msg): # feedback means actual value.
#         for i in range(len(msg.robot_pose_array)):
#             ID=msg.robot_pose_array[i].ID.data
#             self.points[int(ID-2),0]=msg.robot_pose_array[i].position.x * 1.5037594e-3
#             self.points[int(ID-2),1]=msg.robot_pose_array[i].position.y * 1.5306122e-3 
#         self.flag=1
#         self.vector=[]
#         self.vector.append(self.points[1]-self.points[0])
#         self.vector.append(self.points[2]-self.points[3])
#         self.vector.append(self.points[3]-self.points[0])
#         self.vector.append(self.points[2]-self.points[1])
#     def calculateP(self,P):
#         P=P.reshape(1,-1)
#         # weather between vector 0 and 1
#         if np.cross(self.vector[0],P-self.points[0])*np.cross(self.vector[1],P-self.points[3])<=0:
#             if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
#                 self.flag_in=True
#             elif np.cross(self.vector[2],P-self.points[0])>0:
#                 self.flag_in=False
#                 self.senseP[0,:]=self.points[0]
#                 self.senseP[1,:]=self.points[3]
#             else:
#                 self.flag_in=False
#                 self.senseP[0,:]=self.points[1]
#                 self.senseP[1,:]=self.points[2]
#         elif np.cross(self.vector[0],P-self.points[0])<0:
#             self.flag_in=False
#             if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
#                 self.senseP[0,:]=self.points[0]
#                 self.senseP[1,:]=self.points[1]
#             elif np.cross(self.vector[2],P-self.points[0])>0:
#                 self.senseP[0,:]=self.points[3]
#                 self.senseP[1,:]=self.points[1]
#             else:
#                 self.senseP[0,:]=self.points[2]
#                 self.senseP[1,:]=self.points[0]
#         else:
#             self.flag_in=False
#             if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
#                 self.senseP[0,:]=self.points[3]
#                 self.senseP[1,:]=self.points[2]
#             elif np.cross(self.vector[2],P-self.points[0])>0:
#                 self.senseP[0,:]=self.points[0]
#                 self.senseP[1,:]=self.points[2]
#             else:
#                 self.senseP[0,:]=self.points[3]
#                 self.senseP[1,:]=self.points[1]
#         return self.senseP

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
# class K_Force:
#     def __init__(self):
#         self.Ko_tube=np.zeros((N_target,1))
#         self.Ko=np.zeros((2,1))
#         self.sub=rospy.Subscriber('/ForceK1',Float32MultiArray,self.sub_callback,queue_size=10)
#     def sub_callback(self,msg):
#         self.Ko=msg.data[:2]
#         self.Ko_tube=msg.data[2:]

class intersectionPoints:
    def __init__(self):
        self.P=[]
        self.flag_side=[]
        self.sub=rospy.Subscriber('/InterP1',Float32MultiArray,self.sub_callback,queue_size=10)
        self.subflag=rospy.Subscriber('/Flag_side1',Float32MultiArray,self.flag_sub_callback,queue_size=10)

    def sub_callback(self,msg):
        self.P=[]
        for i in range(N_target+2):
           self.P.append(np.array(msg.data[4*i:4*(i+1)]).reshape(2,2))
    
    def flag_sub_callback(self,msg):
        self.flag_side=msg.data

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
# velocity to angular velocity
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

def shift_movement(T, x0, u, f,un,J):
    for i in range(un):
        f_value1 = f(x0[:3], u[i, :2])
        x0[:3]= x0[:3] + T*f_value1.T
        f_value2= f(x0[3:6], u[i, 2:])
        x0[3:6]= x0[3:6] + T*f_value2.T
        x0[-2*N_target:]=x0[-2*N_target:]+ca.mtimes(J,ca.vertcat(T*f_value1[:2].T,T*f_value2[:2].T))
    state_next_=x0
    u_next_ = ca.vertcat(u[un:, :], u[-un:, :])
    return state_next_, u_next_


def my_func(x):
    if x < 0:
        return 0
    else:
        return x

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

class PlotUpdate2():
    #Suppose we know the x range
    #min_x = 0
    #max_x = 300

    def __init__(self,N):
        #Set up plot
        #self.figure, self.ax = plt.subplots()
        self.figure, (self.ax1, self.ax2) = plt.subplots(1, 2, sharey=True)
        self.lines1=[]
        self.lines2=[]
        self.number=N
        self.starttime= time.time()
        self.xdata=[]
        for i in range(self.number):
            line, = self.ax1.plot([],[],label=str(i+1)) #error_x
            self.lines1.append(line)
            line, = self.ax2.plot([],[],label=str(i+1)) #error_x
            self.lines2.append(line)
        #self.lines1, = self.ax1.plot([],[]) #error_x
        #self.lines2, = self.ax2.plot([],[]) #error_y
        #Autoscale on unknown axis and known lims on the other
        self.ax1.set_autoscaley_on(True)
        #self.ax1.set_xlim(self.min_x, self.max_x)
        self.ax2.set_autoscaley_on(True)
        #self.ax2.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax1.grid()
        self.ax2.grid()
        self.ax1.legend(loc='upper left')
        self.ax2.legend(loc='upper left')
        ...
    def on_running(self, error_x, error_y):
        #Update data (with the new _and_ the old points)
        for i in range(self.number):
            timec = time.time()
            self.xdata.append(timec-self.starttime)
            self.lines1[i].set_xdata(self.xdata)
            self.lines1[i].set_ydata(error_x)
            ydata=np.arange(0,len(error_y))
            self.lines2[i].set_xdata(self.xdata)
            self.lines2[i].set_ydata(error_y)
        #Need both of these in order to rescale
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
    def save_plot(self,figname,dataname):
        self.figure.savefig(figname)
        fp = open(dataname, mode='a+', newline='')
        dp = csv.writer(fp)
        dp.writerow(self.lines1[self.number-1].get_xdata())
        dp.writerow(self.lines1[self.number-1].get_ydata())
        dp.writerow(self.lines2[self.number-1].get_xdata())
        dp.writerow(self.lines2[self.number-1].get_ydata())
        fp.close()
        # self.figure.save('error.png')


def calculate_circle_line_intersection(center, radius, startPoint, endPoint):
    # 计算圆与线段的交点
    x, y = symbols('x y', real=True)
    # 圆的方程
    circleEquation = (x - center[0])**2 + (y - center[1])**2 - radius**2
    # 线段的方程
    if startPoint[0] - endPoint[0] != 0:
        lineEquation = (y - startPoint[1]) - ((startPoint[1] - endPoint[1]) / (startPoint[0] - endPoint[0])) * (x - startPoint[0])
    else:
        lineEquation = x - startPoint[0]
    # 求解交点
    intersectionPoints = solve([circleEquation, lineEquation], (x, y))
    # 提取交点坐标
    intersectionPoints = [[float(point[0].evalf()), float(point[1].evalf())] for point in intersectionPoints]
    P = []
    for point in intersectionPoints:
        if ((point[0] - startPoint[0]) * (point[0] - endPoint[0])) <= 0 and ((point[1] - startPoint[1]) * (point[1] - endPoint[1])) <= 0:
            P.append(point)
    return P

def get_intersectionPoints(points,R,V):
    if V[0] < points[1, 0] and V[0] > points[0, 0] and V[1] < points[3, 1] and V[1] > points[0, 1]:
        P=[]
    else:
        # 计算交点
        P = []
        for n in range(3):
            intersectionPoints = calculate_circle_line_intersection(V, R, points[n, :], points[n + 1, :])
            if len(intersectionPoints)!= 0:
                P.append(intersectionPoints)

        intersectionPoints = calculate_circle_line_intersection(V, R, points[3, :], points[0, :])
        if len(intersectionPoints) != 0:
            P.append(intersectionPoints)
        if len(P)==2:
            P=np.reshape(P,(-1,2))
            if P[1,0] == P[0,0] or P[0,1] == P[1,1]:
                Ctheta = np.dot(P[0,:] - V, P[1,:] - V) / np.linalg.norm(P[0,:] - V) / np.linalg.norm(P[1,:] - V)
            else:
                dmin = 1000000
                for m in range(4):
                    if np.linalg.norm(points[m, :] - V) < dmin:
                        dmin = np.linalg.norm(points[m, :] - V)
                        k = m
                # 如果在角
                if (V[0] - points[k, 0]) * (V[0] - P[0,0]) * (V[1] - points[k, 1]) * (V[1] - P[0,1]) > 0 and \
                        (V[0] - points[k, 0]) * (V[0] - P[1,0]) * (V[1] - points[k, 1]) * (V[1] - P[1,1]) > 0:
                    pass
                else:
                    d1 = np.linalg.norm(points[k, :] - P[0,:])
                    d2 = np.linalg.norm(points[k, :] - P[1,:])
                    if d1 > d2:
                        P[0,:] = points[k, :]
                    else:
                        P[1,:] = points[k, :]
    if P!=[]:
        P=np.array(P).reshape(-1,2)    
    return P


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


if __name__ == '__main__':
    try:
        rospy.init_node('mpc_mode')
        QRID=rospy.get_param('~QRID')
        pointname=rospy.get_param('~feature')
        pub = rospy.Publisher('anglevelocity'+str(QRID), Float64MultiArray, queue_size=10)
        enclose_flag_pub=rospy.Publisher('encloseflag'+str(QRID),Bool,queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        feature=Point_tube(pointname)
        Targe_id=Targe_ID(QRID)
        iP=intersectionPoints()
        # Errorplot=PlotUpdate2(1)
       
        # Obstacle=Obstacle_QR()
        # Frame=frame_image()
        T = 0.15# sampling time [s]
        N = 30 # prediction horizon
        un= 4 # control step
        v_max = 0.03
        omega_max = 0.8
        rate = rospy.Rate(10)
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        statesq = ca.vertcat(x, y)
        states = ca.vertcat(statesq, theta)
        n_states = states.size()[0]
        
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        n_controls = controls.size()[0]

        ## rhs
        rhs = ca.horzcat(v*ca.cos(theta)-l_center*ca.sin(theta)*omega, v*ca.sin(theta)+l_center*ca.cos(theta)*omega)
        rhs = ca.horzcat(rhs, omega)

        ## function
        f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        ## for MPC
        U = ca.SX.sym('U', N, n_controls*2)

        X = ca.SX.sym('X', (N+1), n_states*2+2*N_target) # x1,y1,theta1,x2,y1,thate2,xs,ys

        P = ca.SX.sym('P', n_states*2+2*N_target+2)#initial states +target states 2 of s


        ### define
        X[0,:] = P[:n_states*2+2*N_target] # initial condiction
        #J=np.array([[0.5,0,0.5,0],[-0,0.5,-0,0.5]])
        J=np.array([[ 0.28348009 ,-0.10893156, -0.0462114 , -0.06149945],\
                    [ 0.15820577  ,0.3135021 , -0.23376026 , 0.1723733 ],\
                    [ 0.13487716 ,-0.07193869 , 0.16223159 ,-0.02951396],\
                    [ 0.2540872  , 0.27042158 ,-0.35836604 , 0.2319037 ],\
                    [ 0.00593047  ,0.06580107 , 0.30361478 ,-0.03436256],\
                    [ 0.17973368  ,0.22378411 ,-0.3361492  , 0.14395073]])
        ### define the relationship within the horizon
        for i in range(N):
            f_value = f(X[i, :n_states], U[i, :2])
            X[i+1, :n_states] = X[i, :n_states] + f_value*T
            f_value = f(X[i, n_states:2*n_states], U[i, 2:])
            X[i+1, n_states:2*n_states] = X[i, n_states:2*n_states] + f_value*T
            # J matrix
            X[i+1,-2*N_target:]=X[i,-2*N_target:]+ca.mtimes(J,ca.vertcat(X[i+1,:2].T-X[i,:2].T,X[i+1,3:5].T-X[i,3:5].T)).T

        ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

        Q = 1*np.eye(2*N_target)
        R=0.00001*np.eye(4)
        Qr=1*np.eye(2)
        lbx = []
        ubx = []
        for _ in range(N):
            lbx.append(-v_max)
            ubx.append(v_max)
        for _ in range(N):
            lbx.append(-omega_max)
            ubx.append(omega_max)
        for _ in range(N):
            lbx.append(-v_max)
            ubx.append(v_max)
        for _ in range(N):
            lbx.append(-omega_max)
            ubx.append(omega_max)
        # L_real=200* 1.5037594e-3#the length of tube
        # r1=np.array([400* 1.5037594e-3,300* 1.5306122e-3 ])
        # r2=np.array([550* 1.5037594e-3,300* 1.5306122e-3 ])
        # Tube=tubeshape(length=L_real,p1=r1,p2=r2)
        # xp=np.array(Tube.get_points(N_target)).reshape(-1,1)
        # xt=np.concatenate((np.reshape(r1,(2,1)),np.reshape(r2,(2,1)),np.reshape(xp,(len(xp),1))))
        # xs=np.array(xt).reshape(-1,1)
        # xs = np.array([440* 1.5037594e-3, 200* 1.5306122e-3,400* 1.5037594e-3, 220* 1.5306122e-3,360* 1.5037594e-3, 200* 1.5306122e-3 ]).reshape(-1, 1) # final state
        # center=(int(xs[0]),int(xs[1]))
        # cv2.circle(Frame.image, center, 2, (255, 0, 255), -1)
        x_c = [] # contains for the history of the state
        u_c = []
        # t_c = [t0] # for the time
        xx = []
        x_next=None
        error_x=[]
        error_y=[]
        error_d=[]
        error_theta=[]
        # force1_x=[]
        # force1_cos=[]
        # force1_ao=[]
        # force2_x=[]
        # force2_cos=[]
        # force2_ao=[]
        # force_x_tube=[]
        # force_cos_tube=[]
        # force_ao_tube=[]
        u0 = np.array([0,0,0,0]*N).reshape(-1, 4)# np.ones((N, 2)) # controls
        object_flag=0
        r_object=35
        enclose_flag=False
        enclose_flag_pub.publish(enclose_flag)
        while not rospy.is_shutdown():
            
            if Targe_id.flag is True:
                enclose_flag=False
                enclose_flag_pub.publish(enclose_flag)
            ## state 00 begin
            if Robot.flag==1 and feature.middlepoint.x!=0 and Targe_id.transport_flag is False and enclose_flag is False and iP.P is not None:
                # object pick hard constraint
                
                if object_flag==0 and Targe_id.ID!=0 :
                    # deltax=(20+(44-Robot.robotx[5]/1.5037594e-3/25.65))*1.5037594e-3
                    # deltay=(7+(32-Robot.roboty[5]/1.5306122e-3/29.71))*1.5306122e-3
                    IDi=Targe_id.ID-1
                    Target_circle=np.array([Robot.robotx[IDi], Robot.roboty[IDi], r_object* 1.5037594e-3])
                    #plot 
                    Disterror=Plotupdate('Distance'+str(QRID),'distance error (m)',1,['Distance'])
                    Thetaerror=Plotupdate('Theta'+str(QRID),'thera error (rad)',1,['Theta'])
                    # Forceplot1=Plotupdate('Force1','derivative',3,['attrative','steer','push'])
                    # Forceplot2=Plotupdate('Force2','derivative',3,['attrative','steer','push '])
                    # Forceplot_tube=Plotupdate('Force_tube','derivative',3,['attrative ','steer','push '])
                    # flag of obstacle coefficient
                    flag_obstacle=np.zeros((N_target+2,1))
                    Ko=np.zeros((2,1))
                    Ko_tube=np.zeros((N_target,1))
                    xs=[]
                    # for i in range(N_target+2):
                    xs.append(Target_circle[:2])
                    xs=np.array(xs).reshape(-1,1)
                    object_flag=1
                    
                    g = [] # equal constrains
                    lbg = []
                    ubg = []
                    for i in range(N+1):
                        for j in range(8):
                            if j%3==0:
                                g.append(X[i, j])
                                lbg.append(35*1.5306122e-3)
                                ubg.append(1100*1.5306122e-3 )
                            if j%3==1:
                                g.append(X[i, j])
                                lbg.append(35* 1.5037594e-3)
                                ubg.append(430* 1.5037594e-3)
                        for j in range(8,12,1):
                            if j%2==0:
                                g.append(X[i, j])
                                lbg.append(30*1.5306122e-3)
                                ubg.append(1100*1.5306122e-3 )
                            else:
                                g.append(X[i, j])
                                lbg.append(40* 1.5037594e-3)
                                ubg.append(430* 1.5037594e-3)
                    
                    for i in range(N+1):
                        g.append(ca.norm_2(X[i,:2]-X[i,3:5]))
                        lbg.append(L*0.45)
                        ubg.append(L*0.8)


                if object_flag==1:
                   
                # x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],feature.middlepoint.x,feature.middlepoint.y]).reshape(-1, 1)# initial state
                
                    if len(feature.feature_point.points)!=3:
                        x0[:6]=np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]).reshape(-1,1)
                    else:
                        x0= [Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]
                        for xk in range(N_target):
                            x0.append(feature.feature_point.points[xk].x)
                            x0.append(feature.feature_point.points[xk].y)
                    x0=np.array(x0).reshape(-1,1)
                    x1=np.concatenate(x0)
                    # x_center=x0[6:8]
                    # for i in range(1,N_target):
                    #     x_center=x_center+x0[6+2*i:8+2*i]
                    x_center=x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12]
                    x_center=(x_center/(N_target+2)).reshape(1,-1)
                    cos_theta=np.dot(-x0[8:10]+Target_circle[:2],(x0[:2]+x0[3:5])/2-x0[8:10])/np.linalg.norm(-x0[8:10]+Target_circle[:2])/np.linalg.norm((x0[:2]+x0[3:5])/2-x0[8:10])
                    # update enviroment information
                    # R=70*1.5306122e-3
                    ktheta=0.1
                    error_x.append( Target_circle[0]-x_center[0][0])
                    error_y.append( Target_circle[1]-x_center[0][1])
                    error_d.append(np.linalg.norm(Target_circle[:2]-x_center[0]))
                    error_theta.append(math.acos(np.dot(Target_circle[:2]-x1[8:10],(x1[:2]+x1[3:5])/2-x1[8:10])/np.linalg.norm(Target_circle[:2]-x1[8:10])/np.linalg.norm((x1[:2]+x1[3:5])/2-x1[8:10])))
                    # Errorplot.on_running(model_error_x,model_error_y)
                    error_d_update=[error_d]
                    Disterror.on_running(error_d_update)
                    error_theta_update=[error_theta]
                    Thetaerror.on_running(error_theta_update)
                    # if ee[0]>r_object* 1.5037594e-3*1.3 or ee[1]>r_object* 1.5037594e-3*1.3 or ee[2]>r_object* 1.5037594e-3*1.3:
                    if np.linalg.norm(x_center[0]-Target_circle[:2])>r_object* 1.5037594e-3*0.5 and len(iP.P)==5:
                        enclose_flag=False
                        enclose_flag_pub.publish(enclose_flag)
                    # if Obstacle.flag_side is True:
                    #     if Ctheta[i,j]>-0.5:
                    #         F[i,j]=-1
                    #     else:
                    #         F[i,j]=3*Ctheta[i,j]-4*Ctheta[i,j]**3
                    # else:
                    #     if Ctheta[i,j]>0.5:
                    #         F[i,j]=-1
                    #     else:
                    #         F[i,j]=-(-1+2*Ctheta[i,j]**2)*(16*Ctheta[i,j]**4-16*Ctheta[i,j]**2+1)
                        obj = 0 
                        Kt=1000
                        ddp=1.5
                        KO=0.1
                        for i in range(N):
                            obj = obj +  ca.mtimes([U[i, :], R, U[i, :].T])+ca.mtimes([(X[i,:2]+X[i,3:5]+X[i,6:8]+X[i,8:10]+X[i,10:12])/(N_target+2)-P[-2:].T,Qr,((X[i,:2]+X[i,3:5]+X[i,6:8]+X[i,8:10]+X[i,10:12])/(N_target+2)-P[-2:].T).T])-\
                                ktheta*ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2)+\
                                Kt*ca.if_else(ca.norm_2(X[i,:2].T-Target_circle[:2])<Target_circle[2]*(ddp+0.4),(ca.norm_2(X[i,:2].T-Target_circle[:2])-Target_circle[2]*(ddp+0.4))**2,0)+\
                                Kt*ca.if_else(ca.norm_2(X[i,3:5].T-Target_circle[:2])<Target_circle[2]*(ddp+0.4),(ca.norm_2(X[i,3:5].T-Target_circle[:2])-Target_circle[2]*(ddp+0.4))**2,0)
                            for f_tube in range(N_target):
                                obj=obj+Kt*ca.if_else(ca.norm_2(X[i,6+2*f_tube:8+2*f_tube].T-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2(X[i,6+2*f_tube:8+2*f_tube].T-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                                Ctheta_tube=ca.dot(ca.DM(iP.P[2+f_tube][0].reshape(1,-1))- X[i,6+2*f_tube:8+2*f_tube], ca.DM(iP.P[2+f_tube][1].reshape(1,-1)) - X[i,6+2*f_tube:8+2*f_tube])/ca.norm_2(iP.P[2+f_tube][0].reshape(1,-1)- X[i,6+2*f_tube:8+2*f_tube])/ca.norm_2(iP.P[2+f_tube][1].reshape(1,-1) - X[i,6+2*f_tube:8+2*f_tube])
                                # if iP.flag_side[2+f_tube]==1:
                                    # obj=obj+ KO*ca.if_else(Ctheta_tube>-0.5,-1,3*Ctheta_tube-4*Ctheta_tube**3)
                                obj=obj+KO*ca.if_else(Ctheta_tube>0.5,-1,ca.if_else(Ctheta_tube<0,1,-(-1+2*Ctheta_tube**2)*(16*Ctheta_tube**4-16*Ctheta_tube**2+1)))
                            #     else:
                            #         # obj=obj+KO*ca.if_else(Ctheta_tube>0.5,-1,-(-1+2*Ctheta_tube**2)*(16*Ctheta_tube**4-16*Ctheta_tube**2+1))
                            #         obj=obj+KO*ca.if_else(Ctheta_tube>math.sqrt(3)/2,-1,ca.if_else(Ctheta_tube<0.5,1,(-1+2*Ctheta_tube**2)*(16*Ctheta_tube**4-16*Ctheta_tube**2+1)))
                            Ctheta1= ca.dot(ca.DM(iP.P[0][0].reshape(1,-1))- X[i,:2], ca.DM(iP.P[0][1].reshape(1,-1)) - X[i,:2])/ca.norm_2(iP.P[0][0].reshape(1,-1)- X[i,:2])/ca.norm_2(iP.P[0][1].reshape(1,-1) - X[i,:2])
                            Ctheta2= ca.dot(ca.DM(iP.P[1][0].reshape(1,-1))- X[i,3:5], ca.DM(iP.P[1][1].reshape(1,-1)) - X[i,3:5]) /ca.norm_2(iP.P[1][0].reshape(1,-1)- X[i,3:5])/ca.norm_2(iP.P[1][1].reshape(1,-1) - X[i,3:5])
                            # if iP.flag_side[0]==1:
                                # obj=obj+KO*ca.if_else(Ctheta1>-0.5,-1,3*Ctheta1-4*Ctheta1**3)
                            obj=obj+KO*ca.if_else(Ctheta1>0.5,-1,ca.if_else(Ctheta1<0,1,-(-1+2*Ctheta1**2)*(16*Ctheta1**4-16*Ctheta1**2+1)))
                            # else:
                                # obj=obj+KO*ca.if_else(Ctheta1>0.5,-1,-(-1+2*Ctheta1**2)*(16*Ctheta1**4-16*Ctheta1**2+1))
                                # obj=obj+KO*ca.if_else(Ctheta1>math.sqrt(3)/2,-1,ca.if_else(Ctheta1<0.5,1,(-1+2*Ctheta1**2)*(16*Ctheta1**4-16*Ctheta1**2+1)))
                            # if iP.flag_side[1]==1:
                                # obj=obj+KO*ca.if_else(Ctheta2>-0.5,-1,3*Ctheta2-4*Ctheta2**3)
                            obj=obj+KO*ca.if_else(Ctheta2>0.5,-1,ca.if_else(Ctheta2<0,1,-(-1+2*Ctheta2**2)*(16*Ctheta2**4-16*Ctheta2**2+1)))
                            # else:
                            #     # obj=obj+KO*ca.if_else(Ctheta2>0.5,-1,-(-1+2*Ctheta2**2)*(16*Ctheta2**4-16*Ctheta2**2+1)) 
                            #     obj=obj+KO*ca.if_else(Ctheta2>math.sqrt(3)/2,-1,ca.if_else(Ctheta2<0.5,1,(-1+2*Ctheta2**2)*(16*Ctheta2**4-16*Ctheta2**2+1)))
                            
                        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
                        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
                        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
    
                        ## set parameter
                        c_p = np.concatenate((x0, xs))
                        init_control = ca.reshape(u0, -1, 1)
                        t_ = time.time()
                        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
                        # index_t.append(time.time()- t_)
                        u_sol = ca.reshape(res['x'],  N, n_controls*2) # one can only have this shape of the output
                        # ff_value = ff(u_sol, c_p) # [n_states, N]
                        x_next, u0 = shift_movement(T, x0, u_sol, f,un,J)
                        # x_c.append(ff_value)
                        u_c.append(np.array(u_sol.full()))
                        xx.append(x0.tolist())
                        vel=v2w(np.array(u_sol.full()),un)
                        for i in range(un):
                            vel_msg = Float64MultiArray(data=vel[i,:])
                            rospy.loginfo(vel_msg)
                            pub.publish(vel_msg)
                            d = rospy.Duration(T)
                            rospy.sleep(d)
                        ran_vel=np.zeros((1,4))
                        vel_msg = Float64MultiArray(data=ran_vel[0])
                        rospy.loginfo(vel_msg)
                        pub.publish(vel_msg)
                        d = rospy.Duration(0.0001)
                        rospy.sleep(d)

                        
                    else:
                        # save data
                        # Errorplot.save_plot('error.png','error.csv')
                        Disterror.save_plot('distanceerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','distance'+str(Targe_id.ID)+'.csv')
                        Thetaerror.save_plot('thetaerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','theta'+str(Targe_id.ID)+'.csv')
                        # Forceplot1.save_plot('Force1'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Force1'+str(Targe_id.ID)+'.csv')
                        # Forceplot2.save_plot('Force2'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Force2'+str(Targe_id.ID)+'.csv')
                        # Forceplot_tube.save_plot('Force_tube'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Force_tube'+str(Targe_id.ID)+'.csv')
                        ran_vel=np.zeros((1,4))
                        vel_msg = Float64MultiArray(data=ran_vel[0])
                        rospy.loginfo(vel_msg)
                        pub.publish(vel_msg)
                        d = rospy.Duration(0.005)
                        rospy.sleep(d)
                        object_flag=0
                        enclose_flag=True
                        enclose_flag_pub.publish(enclose_flag)


    except rospy.ROSInterruptException:
        pass
