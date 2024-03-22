#!/usr/bin/env python3
# use the Aruco code  detect the tube feature points
# calculate the derivative to design the weight 
from std_msgs.msg import Int8
from std_msgs.msg import Bool
import rospy
from std_msgs.msg import String
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
# import env
# from draw import Draw_MPC_two_agents_withtube
import matplotlib.pyplot as plt
import csv
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
# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=3 # feature points number
plt.ion()

def point_line_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1] 
    d = dx*dx + dy*dy
    t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / d
    if d:
        if t < 0:
            p = start
        elif t > 1:
            p = end
        else:
            p = np.array([start[0]+ t * dx, start[1] + t * dy])
    else:
            p = start
        
    dx = point[0]- p[0]
    dy = point[1] - p[1]
    distence=math.sqrt(dx*dx + dy*dy)
    return [distence,p]


class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.sub = rospy.Subscriber('/feature_points', PoseArray, self.tube_callback,queue_size=10)
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

class Obstacle_QR:
    def __init__(self):
        self.points=np.zeros((4,2))
        self.flag=0
        self.senseP=np.zeros((2,2))
        self.flag_in=True
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
        P=P.reshape(1,-1)
        # weather between vector 0 and 1
        if np.cross(self.vector[0],P-self.points[0])*np.cross(self.vector[1],P-self.points[3])<=0:
            if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
                self.flag_in=True
            elif np.cross(self.vector[2],P-self.points[0])>0:
                self.flag_in=False
                self.senseP[0,:]=self.points[0]
                self.senseP[1,:]=self.points[3]
            else:
                self.flag_in=False
                self.senseP[0,:]=self.points[1]
                self.senseP[1,:]=self.points[2]
        elif np.cross(self.vector[0],P-self.points[0])<0:
            self.flag_in=False
            if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
                self.senseP[0,:]=self.points[0]
                self.senseP[1,:]=self.points[1]
            elif np.cross(self.vector[2],P-self.points[0])>0:
                self.senseP[0,:]=self.points[3]
                self.senseP[1,:]=self.points[1]
            else:
                self.senseP[0,:]=self.points[2]
                self.senseP[1,:]=self.points[0]
        else:
            self.flag_in=False
            if np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
                self.senseP[0,:]=self.points[3]
                self.senseP[1,:]=self.points[2]
            elif np.cross(self.vector[2],P-self.points[0])>0:
                self.senseP[0,:]=self.points[0]
                self.senseP[1,:]=self.points[2]
            else:
                self.senseP[0,:]=self.points[3]
                self.senseP[1,:]=self.points[1]
        return self.senseP

    def getnearestpoint(self,P):
        if np.cross(self.vector[0],P-self.points[0])*np.cross(self.vector[1],P-self.points[3])<=0 and np.cross(self.vector[2],P-self.points[0])*np.cross(self.vector[3],P-self.points[1])<=0:
            self.flag_in=True
        else:
            self.flag_in=False
        d=np.zeros((4,1))
        p=np.zeros((4,2))
        for i in range(3):
            [d[i],p[i,:]]=point_line_distance(P, self.points[i], self.points[i+1])
        [d[3],p[3,:]]=point_line_distance(P, self.points[3], self.points[0])
        min_value = np.min(d)
        min_index = np.argmin(d)
        if self.flag_in is True:
            min_value=-min_value
        return [min_value,p[min_index,:]]
    def EGO(self,sf,d):
        c=sf-d
        if c<=0:
            EGOJ=0
        elif c>0 and c<=sf:
            EGOJ=c**3
        else:
            EGOJ=3*sf*c**2-3*sf**2*c+sf**3
        return EGOJ

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
    def __init__(self):
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


# class PlotUpdate():
#     #Suppose we know the x range
#     #min_x = 0
#     #max_x = 300

#     def __init__(self,N):
#         #Set up plot
#         #self.figure, self.ax = plt.subplots()
#         self.figure, (self.ax1, self.ax2) = plt.subplots(1, 2, sharey=True)
#         self.lines1=[]
#         self.lines2=[]
#         self.number=N
#         for i in range(self.number):
#             line, = self.ax1.plot([],[],label=str(i+1)) #error_x
#             self.lines1.append(line)
#             line, = self.ax2.plot([],[],label=str(i+1)) #error_x
#             self.lines2.append(line)
#         #self.lines1, = self.ax1.plot([],[]) #error_x
#         #self.lines2, = self.ax2.plot([],[]) #error_y
#         #Autoscale on unknown axis and known lims on the other
#         self.ax1.set_autoscaley_on(True)
#         #self.ax1.set_xlim(self.min_x, self.max_x)
#         self.ax2.set_autoscaley_on(True)
#         #self.ax2.set_xlim(self.min_x, self.max_x)
#         #Other stuff
#         self.ax1.grid()
#         self.ax2.grid()
#         self.ax1.legend(loc='upper left')
#         self.ax2.legend(loc='upper left')
#         ...
#     def on_running(self, error_x, error_y):
#         #Update data (with the new _and_ the old points)
#         for i in range(self.number):
#             xdata=np.arange(0,len(error_x))
#             self.lines1[i].set_xdata(xdata)
#             self.lines1[i].set_ydata(error_x)
#             ydata=np.arange(0,len(error_y))
#             self.lines2[i].set_xdata(ydata)
#             self.lines2[i].set_ydata(error_y)
#         #Need both of these in order to rescale
#         self.ax1.relim()
#         self.ax1.autoscale_view()
#         self.ax2.relim()
#         self.ax2.autoscale_view()
#         #We need to draw *and* flush
#         self.figure.canvas.draw()
#         self.figure.canvas.flush_events()


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

def getdcos_to_x1(X1,X2,qm,pt,J1,J2):
    l1=np.linalg.norm(pt-qm)
    l2=np.linalg.norm((X1+X2)/2-qm)
    z1=(pt-qm)/l1
    z2=((X1+X2)/2-qm)/l2
    I=np.eye((2))
    M1=np.matmul(I- np.matmul(z1,np.transpose(z1))/l1,-J1)
    M2=np.matmul(I- np.matmul(z2,np.transpose(z2))/l2,(I/2))
    D=np.matmul(np.transpose(z1),M2)
    return D

def distence_to_x(pv,pt,Qr):
    D=np.matmul(np.transpose(pv-pt),(Qr+np.transpose(Qr)))/(N_target+2)
    return D


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

class KeyboardSubscriber:
    def __init__(self):
        self.state=True
        self.sub=rospy.Subscriber('/keypress', String,self.Key_callback, queue_size=10)
    def Key_callback(self,msg):
        s="'q'"
        if msg.data==s:
            self.state=False
        print(msg.data+'received')


if __name__ == '__main__':
    try:
        rospy.init_node('mpc_mode')
        QRID=rospy.get_param('~QRID')
        pointname=rospy.get_param('~feature')
        pub = rospy.Publisher('anglevelocity'+str(QRID), Float64MultiArray, queue_size=10)
        enclose_flag_pub=rospy.Publisher('encloseflag'+str(QRID),Bool,queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        feature=Point_tube()
        Targe_id=Targe_ID()
        # model_errorplot=PlotUpdate(1)
        Obstacle=Obstacle_QR()
        key=KeyboardSubscriber()
        save_flag=0
        # Frame=frame_image()
        T = 0.15# sampling time [s]
        N = 30 # prediction horizon
        un= 4 # control step
        v_max = 0.025
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
        R=0.001*np.eye(4)
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
        model_error_x=[]
        model_error_y=[]
        u0 = np.array([0,0,0,0]*N).reshape(-1, 4)# np.ones((N, 2)) # controls
        object_flag=0
        r_object=32
        enclose_flag=False
        enclose_flag_pub.publish(enclose_flag)
        while not rospy.is_shutdown():
            
            if Targe_id.flag is True:
                enclose_flag=False
                enclose_flag_pub.publish(enclose_flag)
            ## state 00 begin
            if Robot.flag==1 and feature.middlepoint.x!=0 and Targe_id.transport_flag is False and enclose_flag is False and Obstacle.flag==1 :
                # object pick hard constraint
                
                if object_flag==0 and Targe_id.ID!=0 :
                    # deltax=(20+(44-Robot.robotx[5]/1.5037594e-3/25.65))*1.5037594e-3
                    # deltay=(7+(32-Robot.roboty[5]/1.5306122e-3/29.71))*1.5306122e-3
                    IDi=Targe_id.ID-1
                    Target_circle=np.array([Robot.robotx[IDi], Robot.roboty[IDi], r_object* 1.5037594e-3])
                    Cal_time=Plotupdate('Time'+str(QRID),'calculate time (s)',1,['Time'])
                    #plot 
                    Disterror=Plotupdate('Distance'+str(QRID),'distance error (m)',1,['Distance'])
                    Thetaerror=Plotupdate('Theta'+str(QRID),'thera error (rad)',1,['Theta'])
                    Jerror=Plotupdate('model'+str(QRID),'model error (m)',1,['Distance'])
                    calculate_time=[]
                    save_flag=0
                    error_x=[]
                    error_y=[]
                    error_d=[]
                    error_theta=[]
                    error_model=[]
                    xs=[]
                    # for i in range(N_target+2):
                    xs.append(Target_circle[:2])
                    xs=np.array(xs).reshape(-1,1)
                    object_flag=1
                    cos_flag=0
                    g = [] # equal constrains
                    lbg = []
                    ubg = []
                    for i in range(N+1):
                        for j in range(8):
                            if j%3==0:
                                g.append(X[i, j])
                                lbg.append(30*1.5306122e-3)
                                ubg.append(1100*1.5306122e-3 )
                            if j%3==1:
                                g.append(X[i, j])
                                lbg.append(35* 1.5037594e-3)
                                ubg.append(450* 1.5037594e-3)
                        for j in range(8,12,1):
                            if j%2==0:
                                g.append(X[i, j])
                                lbg.append(30*1.5306122e-3)
                                ubg.append(1100*1.5306122e-3 )
                            else:
                                g.append(X[i, j])
                                lbg.append(35* 1.5037594e-3)
                                ubg.append(450* 1.5037594e-3)
                    ddp=1.5
                    for i in range(N+1):
                        g.append(ca.norm_2(X[i,:2]-X[i,3:5]))
                        lbg.append(L*0.5)
                        ubg.append(L*0.9)
                        g.append(ca.norm_2(X[i,:2].T-Target_circle[:2]))
                        lbg.append(Target_circle[2]*2)
                        ubg.append(200)
                        g.append(ca.norm_2(X[i,3:5].T-Target_circle[:2]))
                        lbg.append(Target_circle[2]*2)
                        ubg.append(200)
                        g.append(ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2))
                        lbg.append(-0.5)
                        ubg.append(200)
                        for j in range(N_target):
                            g.append(ca.norm_2(X[i,6+2*j:8+2*j].T-Target_circle[:2]))
                            lbg.append(Target_circle[2]*ddp)
                            ubg.append(200)
                        g.append(ca.norm_2((X[i,:2].T+X[i,6:8].T)/2-Target_circle[:2]))
                        lbg.append(Target_circle[2]*ddp)
                        ubg.append(200)
                        g.append(ca.norm_2((X[i,6:8].T+X[i,8:10].T)/2-Target_circle[:2]))
                        lbg.append(Target_circle[2]*ddp)
                        ubg.append(200)
                        g.append(ca.norm_2((X[i,8:10].T+X[i,10:12].T)/2-Target_circle[:2]))
                        lbg.append(Target_circle[2]*ddp)
                        ubg.append(200)
                        g.append(ca.norm_2((X[i,10:12].T+X[i,3:5].T)/2-Target_circle[:2]))
                        lbg.append(Target_circle[2]*ddp)
                        ubg.append(200)
                    for a in range(len(Robot.robotID)):
                        if Robot.robotID[a]>2 and Robot.robotID[a]<10 and Robot.robotID[a]!=Targe_id.ID:
                            C=np.array([Robot.robotx[a], Robot.roboty[a], r_object* 1.5037594e-3])
                            for i in range(N+1):
                                g.append(ca.norm_2(X[i,:2].T-C[:2]))
                                lbg.append(C[2]*2)
                                ubg.append(200)
                                g.append(ca.norm_2(X[i,3:5].T-C[:2]))
                                lbg.append(C[2]*2)
                                ubg.append(200)
                                g.append(ca.norm_2((X[i,6:8].T+X[i,10:12].T+X[i,8:10].T)/3-C[:2]))
                                lbg.append(C[2]*2)
                                ubg.append(200)
                                for j in range(N_target):
                                    g.append(ca.norm_2(X[i,6+2*j:8+2*j].T-C[:2]))
                                    lbg.append(C[2]*ddp)
                                    ubg.append(200)

                   


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
                    if cos_flag==0 and np.linalg.norm(Target_circle[:2]-x_center)<400*1.5037594e-3:
                        cos_flag=1
                    # update enviroment information
                    # R=70*1.5306122e-3
                    ktheta=1
                    Kt=10000
                    # if ee[0]>r_object* 1.5037594e-3*1.3 or ee[1]>r_object* 1.5037594e-3*1.3 or ee[2]>r_object* 1.5037594e-3*1.3:
                    if np.linalg.norm(x_center[0]-Target_circle[:2])>r_object* 1.5037594e-3*0.5:
                        enclose_flag=False
                        enclose_flag_pub.publish(enclose_flag)               

                        Sf=65* 1.5037594e-3
                        
                        # x1=np.concatenate(x0)
                        intersectionP=[]
                        intersectionP.append(Obstacle.getnearestpoint(x1[:2])[1])
                        intersectionP.append(Obstacle.getnearestpoint(x1[3:5])[1])
                        intersectionP.append(Obstacle.getnearestpoint(x1[6:8])[1])
                        intersectionP.append(Obstacle.getnearestpoint(x1[8:10])[1])
                        intersectionP.append(Obstacle.getnearestpoint(x1[10:12])[1])
                        start_time = time.time()
                        #the middle point of the tube
                        DO_tube=[]
                        Dx=distence_to_x((x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12])/(2+N_target),xs,Qr)
                        Ko=np.zeros((2,1))
                        Ko_tube=np.zeros((N_target,1))
                        for f_tube in range(N_target):
                            dist=Sf-np.linalg.norm(x1[6+2*f_tube:8+2*f_tube]-intersectionP[2+f_tube])
                            if dist<Sf:
                                Ko_tube[f_tube]=np.linalg.norm(Dx)/np.linalg.norm(Obstacle.EGO(Sf,dist))
                            else:
                                Ko_tube[f_tube]=0

                        D=getdcos_to_x1(x0[:2],x0[3:5],x0[8:10],xs,J[2:4,:2],J[2:4,-2:])
                        
                        # for f_tube in range(N_target):
                        #     DC=np.dot(Dtanh_tube[f_tube],J[2*f_tube:2+2*f_tube,2*io:2*io+2])
                        #     Ko_tube[f_tube,io]=np.linalg.norm(Dx[io])/np.linalg.norm(DC)
                        obj = 0 #### cost
                        for i in range(N):
                            obj = obj +  ca.mtimes([U[i, :], R, U[i, :].T])+ca.mtimes([(X[i,:2]+X[i,3:5]+X[i,6:8]+X[i,8:10]+X[i,10:12])/(N_target+2)-P[-2:].T,Qr,((X[i,:2]+X[i,3:5]+X[i,6:8]+X[i,8:10]+X[i,10:12])/(N_target+2)-P[-2:].T).T])-\
                                ktheta*ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2)+\
                                Kt*ca.if_else(ca.norm_2(X[i,:2].T-Target_circle[:2])<Target_circle[2]*(ddp+0.5),(ca.norm_2(X[i,:2].T-Target_circle[:2])-Target_circle[2]*(ddp+0.5))**2,0)+\
                                Kt*ca.if_else(ca.norm_2(X[i,3:5].T-Target_circle[:2])<Target_circle[2]*(ddp+0.5),(ca.norm_2(X[i,3:5].T-Target_circle[:2])-Target_circle[2]*(ddp+0.5))**2,0)
                            if cos_flag==1:
                                obj=obj-ktheta*ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2)
                            for f_tube in range(N_target):
                                obj=obj+Kt*ca.if_else(ca.norm_2(X[i,6+2*f_tube:8+2*f_tube].T-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2(X[i,6+2*f_tube:8+2*f_tube].T-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                            obj=obj+Kt*ca.if_else(ca.norm_2((X[i,3:5].T+X[i,10:].T)/2-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2((X[i,3:5].T+X[i,10:].T)/2-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                            obj=obj+Kt*ca.if_else(ca.norm_2((X[i,:2].T+X[i,6:8].T)/2-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2((X[i,:2].T+X[i,6:8].T)/2-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                            # beta scaled
                        # for i in range(un):
                            obj=obj+10000000*ca.if_else(ca.norm_2(ca.DM(intersectionP[0][0].reshape(1,-1))- X[i,:2])>Sf,0,(-1/Sf+1/ca.norm_2(ca.DM(intersectionP[0][0].reshape(1,-1))- X[i,:2]))**2)\
                                +10000000*ca.if_else(ca.norm_2(ca.DM(intersectionP[1][0].reshape(1,-1))- X[i,3:5])>Sf,0,(-1/Sf+1/ca.norm_2(ca.DM(intersectionP[1][0].reshape(1,-1))- X[i,3:5]))**2)
                            for j in range(3):
                                obj=obj+10000000*ca.if_else(ca.norm_2(ca.DM(intersectionP[j+2][0].reshape(1,-1))- X[i,6+2*j:8+2*j])>Sf,0,(-1/Sf+1/ca.norm_2(ca.DM(intersectionP[j+2][0].reshape(1,-1))- X[i,6+2*j:8+2*j]))**2) 
                                # Ko[0]*ca.tanh(a*(ca.dot(ca.DM(intersectionP[0][0].reshape(1,-1))- X[i,:2], ca.DM(intersectionP[0][1].reshape(1,-1)) - X[i,:2]) / ca.norm_2(ca.DM(intersectionP[0][0].reshape(1,-1)) - X[i,:2]) / ca.norm_2(ca.DM(intersectionP[0][1].reshape(1,-1))- X[i,:2]))+b)-\
                                # Ko[1]*ca.tanh(a*(ca.dot(ca.DM(intersectionP[1][0].reshape(1,-1))- X[i,3:5], ca.DM(intersectionP[1][1].reshape(1,-1)) - X[i,3:5]) / ca.norm_2(ca.DM(intersectionP[1][0].reshape(1,-1)) - X[i,3:5]) / ca.norm_2(ca.DM(intersectionP[1][1].reshape(1,-1)) - X[i,3:5]))+b)
                            # for f_tube in range(N_target):
                            #     obj = obj -(Ko_tube[f_tube])*ca.tanh(a*(ca.dot(ca.DM(intersectionP[2+f_tube][0].reshape(1,-1))- X[i,6+2*f_tube:8+2*f_tube], ca.DM(intersectionP[2+f_tube][1].reshape(1,-1)) - X[i,6+2*f_tube:8+2*f_tube]) / ca.norm_2(ca.DM(intersectionP[2+f_tube][0].reshape(1,-1)) - X[i,6+2*f_tube:8+2*f_tube]) / ca.norm_2(ca.DM(intersectionP[2+f_tube][1].reshape(1,-1))- X[i,6+2*f_tube:8+2*f_tube]) )+b)
                            # if np.linalg.norm(x_center[0]-Target_circle[:2])/1.5037594e-3 <200:
                            #     obj=obj-1*ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2)
                            # if len(intersectionP[0])==2: 
                            #     PI=ca.DM(np.array(intersectionP[0]).reshape(-1,2))
                            #     obj=obj-kcos*ca.tanh(3/(1+Sf)*(ca.dot(PI[0,:]-X[i,:2],PI[1,:]-X[i,:2])/ca.norm_2(PI[0,:]-X[i,:2])/ca.norm_2(PI[1,:]-X[i,:2])+1))
                            # else:
                            #     obj=obj-kcos
                            # if len(intersectionP[1])==2:
                            #     PI=ca.DM(np.array(intersectionP[1]).reshape(-1,2))
                            #     obj=obj-kcos*ca.tanh(3/(1+Sf)*(ca.dot(PI[0,:]-X[i,3:5],PI[1,:]-X[i,3:5])/ca.norm_2(PI[0,:]-X[i,3:5])/ca.norm_2(PI[1,:]-X[i,3:5])+1))
                            # else:
                            #     obj=obj-kcos
                            # if len(intersectionP[2])==2:
                            #     PI=ca.DM(np.array(intersectionP[2]).reshape(-1,2))
                            #     obj=obj-kcos*ca.tanh(3/(1+Sf)*(ca.dot(PI[0,:]-X[i,8:10],PI[1,:]-X[i,8:10])/ca.norm_2(PI[0,:]-X[i,8:10])/ca.norm_2(PI[1,:]-X[i,8:10])+1))
                            # else:
                            #     obj=obj-kcos
                    
                        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
                        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
                        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
                        
                        ## set parameter
                        c_p = np.concatenate((x0, xs))
                        init_control = ca.reshape(u0, -1, 1)
                        t_ = time.time()
                        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
                        # index_t.append(time.time()- t_)
                        
                        elapsed_time = time.time() - start_time
                        print("Execution time: {:.2f} seconds".format(elapsed_time))
                        calculate_time.append(elapsed_time)
                        calculate_time_update=[calculate_time]
                        Cal_time.on_running(calculate_time_update)
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
                        d = rospy.Duration(0.001)
                        rospy.sleep(d)
                        if key.state is False and save_flag==0:
                            save_flag=1
                            Cal_time.save_plot('Cal_time_EGO'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Cal_time'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                            # Disterror.save_plot('EGOdistanceerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','EGOdistance'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                            # Thetaerror.save_plot('EGOthetaerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','EGOtheta'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                            # Jerror.save_plot('EGOerror_model'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','EGOmodelerror'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')

                        
                    else:
                        Cal_time.save_plot('Cal_time_EGO'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Cal_time'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                        # ran_vel=np.zeros((1,4))
                        # vel_msg = Float64MultiArray(data=ran_vel[0])
                        # rospy.loginfo(vel_msg)
                        # pub.publish(vel_msg)
                        # d = rospy.Duration(0.005)
                        # rospy.sleep(d)
                        object_flag=0
                        enclose_flag=True
                        enclose_flag_pub.publish(enclose_flag)


    except rospy.ROSInterruptException:
        pass
