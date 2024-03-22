#!/usr/bin/env python3
# use the Aruco code  detect the tube feature points
# final version with obstacle avoidance
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
import rospy
from std_msgs.msg import String
from pynput import keyboard
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
from pynput import keyboard
from numpy import linalg as LA
import csv
import signal
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
from scipy.special import comb
# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=3 # feature points number
plt.ion()

def exit(signum,frame):
    print('stop')
    exit()

class Jmatrix():
    def __init__(self,P:int):
        # self.J=np.array([[0.5,0,0.5,0],[0,0.5,0,0.5]])
        self.J=1*np.ones((P,4))
        self.p1=np.array((2,1))
        self.p2=np.array((2,1))
        self.e=1e-6 #error
        self.j=0
        self.points=np.array((P,1))
        self.datanumber=P # the number of save date
        self.xdata=np.zeros((1,4))
        self.ydata=np.zeros((1,P))
    
    def initialJ(self,r1,r2,points):
        self.p1=r1
        self.p2=r2
        self.points=points 

    def unpdateJ(self,p1,p2,xtn):
        gamma=100
        deltap=np.concatenate(((p1-self.p1).reshape(1,2),(p2-self.p2).reshape(1,2)), axis=1)
        self.p1=p1
        self.p2=p2
        # self.Tube.update(p1,p2)
        # # the change of tube
        # xtn=np.array(self.Tube.get_points(self.n_tube)).ravel().reshape(2*self.n_tube,1)
        #delta s
        delta_t=xtn-self.points
        self.points=xtn
        if LA.norm(deltap)!=0 and LA.norm(delta_t)!=0:
            for i in range(len(self.points)):
                self.j=LA.norm(np.dot(deltap.reshape(1,4),self.J[i,:].reshape(4,1))-delta_t[i])**2/2+LA.norm(np.dot(self.xdata,self.J[i,:].reshape(4,1))-self.ydata[:,i].reshape(-1,1))**2/2
                if self.j>self.e:
                    qn=self.J[i,:].reshape(4,1)\
                    -gamma*(np.dot(self.xdata.T,np.dot(self.xdata,self.J[i,:].reshape(4,1))-self.ydata[:,i].reshape(-1,1))+np.dot(deltap.reshape(4,1),np.dot(deltap.reshape(1,4),self.J[i,:].reshape(4,1))-delta_t[i]).reshape(4,1))
                    self.J[i,:]=qn.reshape(1,4)
            # update R
            self.xdata=np.vstack((self.xdata[len(deltap):,:],deltap))
            #update delta
            self.ydata=np.vstack((self.ydata[len(delta_t.T):,:],delta_t.T))


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




def on_press(key, abortKey='esc'):    
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys    

    print('pressed %s' % (k))
    if k == abortKey:
        print('end loop ...')
        return False  # stop listener



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
        self.points=np.zeros((len(msg.robot_pose_array),2))
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

def getdcos_to_x(X1,X2,qm,pt,J1,J2):
    l1=np.linalg.norm(pt-qm)
    l2=np.linalg.norm((X1+X2)/2-qm)
    z1=(pt-qm)/l1
    z2=((X1+X2)/2-qm)/l2
    I=np.eye((2))
    M1=np.matmul(I- np.matmul(z1,np.transpose(z1))/l1,-J1)
    M2=np.matmul(I- np.matmul(z2,np.transpose(z2))/l2,(I/2))
    D1=np.matmul(np.transpose(z2),M1)+np.matmul(np.transpose(z1),M2)
    M1=np.matmul(I- np.matmul(z1,np.transpose(z1))/l1,-J2)
    D2=np.matmul(np.transpose(z2),M1)+np.matmul(np.transpose(z1),M2)
    return D1,D2
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

def distence_to_x(pv,pt,Qr,J):
    I=np.eye((2))
    D=np.matmul(np.transpose(pv-pt),(Qr+np.transpose(Qr)))/(N_target+2)
    d1=np.matmul(D,(I+J[:2,:2]+J[2:4,:2]+J[4:6,:2])/5)
    d2=np.matmul(D,(I+J[:2,-2:]+J[2:4,-2:]+J[4:6,-2:])/5)
    return d1,d2

def distance_point_to_segment(point, segment_start, segment_end):
    # 计算线段的向量
    segment_vector = segment_end - segment_start

    # 计算从起点到点的向量
    point_vector = point - segment_start

    # 计算线段长度的平方
    segment_length_sq = np.dot(segment_vector, segment_vector)

    # 如果线段长度的平方接近于0，则点和线段重合
    if segment_length_sq < 1e-6:
        return np.linalg.norm(point_vector)

    # 计算点到线段的投影比例
    projection_ratio = np.dot(point_vector, segment_vector) / segment_length_sq

    # 如果投影比例小于0，则点在线段起点之前
    if projection_ratio < 0:
        return np.linalg.norm(point_vector), segment_start

    # 如果投影比例大于1，则点在线段终点之后
    if projection_ratio > 1:
        return np.linalg.norm(point - segment_end), segment_end

    # 计算点到线段的垂直距离
    perpendicular_distance = np.linalg.norm(point_vector - projection_ratio * segment_vector)
    closest=  segment_start+ segment_vector*projection_ratio

    return perpendicular_distance, closest

def distance_point_to_polygon(point, polygon):
    min_distance = math.inf
    # 遍历多边形的边
    for i in range(len(polygon)):
        segment_start = polygon[i]
        segment_end = polygon[(i + 1) % len(polygon)]
        [distance, closest] = distance_point_to_segment(point, segment_start, segment_end)
        if distance < min_distance:
            min_distance = distance
            min_point=closest

    return min_distance,min_point


class KeyboardSubscriber:
    def __init__(self):
        self.state=True
        self.sub=rospy.Subscriber('/keypress', String,self.Key_callback, queue_size=10)
    def Key_callback(self,msg):
        s="'q'"
        if msg.data==s:
            self.state=False
        print(msg.data+'received')



class Avoidance_scale:
    def __init__(self):
        self.id=-1
        self.sub=rospy.Subscriber('/alphaID1',Int8,self.ID_callback,queue_size=10)
    def ID_callback(self,msg):
        self.id=msg.data

def bpoly(n, t, k):
        """ Bernstein polynomial when a = 0 and b = 1. """
        return t ** k * (1 - t) ** (n - k) * comb(n, k)
        #return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

def bmatrix(T,degree):
    """ Bernstein matrix for Bézier curves. """
    return np.matrix([[bpoly(degree, t, k) for k in range(degree + 1)] for t in T])

class SVM_plane:
    def __init__(self):
        self.w=np.zeros((1,2))
        self.b=0
        self.ID=[]
        self.wb_sub=rospy.Subscriber('/wb',Point32,self.wb_callback,queue_size=10)
        self.ID_sub=rospy.Subscriber('/IDarray',Int32MultiArray,self.ID_callback,queue_size=10)
    def wb_callback(self,msg):
        self.w[0,0]=msg.x
        self.w[0,1]=msg.y
        self.b=msg.z
    def ID_callback(self,msg):
        self.ID=[]
        for i in range(len(msg.data)):
            self.ID.append(msg.data[i])
 
 # generate FF
def gene_ff(X,U,P,f,J):
    ### define
    X[0,:] = P[:n_states*2+2*N_target] # initial condiction
    #### define the relationship within the horizon
    for i in range(N):
        f_value = f(X[i, :n_states], U[i, :2])
        X[i+1, :n_states] = X[i, :n_states] + f_value*T
        f_value = f(X[i, n_states:n_states+3], U[i, 2:])
        X[i+1, n_states:n_states+3] = X[i, n_states:n_states+3] + f_value*T
        # J matrix
        X[i+1,-N_target*2:]=X[i,-N_target*2:]+ca.mtimes(J,ca.vertcat(X[i+1,:2].T-X[i,:2].T,X[i+1,3:5].T-X[i,3:5].T)).T

    ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

    return [X,ff]

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
        alpha=Avoidance_scale()
        # Obstacle=Obstacle_QR()
        Hepyplane=SVM_plane()
        key=KeyboardSubscriber()
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
        JM=Jmatrix(P=N_target*2)
        [X,ff]=gene_ff(X,U,P,f,JM.J)

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
        x_c = [] # contains for the history of the state
        u_c = []
        # t_c = [t0] # for the time
        xx = []
        x_next=None
        
        u0 = np.array([0,0,0,0]*N).reshape(-1, 4)# np.ones((N, 2)) # controls
        object_flag=0
        r_object=40
        enclose_flag=False
        enclose_flag_pub.publish(enclose_flag)
        if QRID==1:
            RID=0
        else:
            RID=2

        while not rospy.is_shutdown():
 
           
            if Targe_id.flag is True:
                enclose_flag=False
                enclose_flag_pub.publish(enclose_flag)
            ## state 00 begin
            if Robot.flag==1 and feature.middlepoint.x!=0 and Targe_id.transport_flag is False and enclose_flag is False and iP.P is not None:
                # object pick hard constraint
                if object_flag==0 and Targe_id.ID!=0 :
                    
                    IDi=Targe_id.ID-1
                    Target_circle=np.array([Robot.robotx[IDi], Robot.roboty[IDi], r_object* 1.5037594e-3])
                    # polygon=[]
                    # for io in range(Obstacle.points.shape[0]):
                    #     polygon.append(Obstacle.points[io])
                    
                    #plot 
                    Disterror=Plotupdate('Distance'+str(QRID),'distance error (m)',1,['Distance'])
                    Thetaerror=Plotupdate('Theta'+str(QRID),'thera error (rad)',1,['Theta'])
                    Jerror=Plotupdate('model'+str(QRID),'model error (m)',1,['Distance'])
                    Cal_time=Plotupdate('Time'+str(QRID),'calculate time (s)',1,['Time'])
                    save_flag=0
                    error_x=[]
                    error_y=[]
                    error_d=[]
                    error_theta=[]
                    error_model=[]
                    calculate_time=[]
                    # flag of obstacle coefficient
                    flag_obstacle=np.zeros((N_target+2,1))
                    Ko=np.zeros((2,1))
                    Ko_tube=np.zeros((N_target,1))
                    xs=[]
                    # for i in range(N_target+2):
                    xs.append(Target_circle[:2])
                    xs=np.array(xs).reshape(-1,1)

                    r1=np.array([[Robot.robotx[RID+0]],[Robot.roboty[RID+0]]])
                    r2=np.array([[Robot.robotx[RID+1]],[Robot.roboty[RID+1]]])
                    points=[]
                    for xk in range(N_target):
                        points.append(feature.feature_point.points[xk].x)
                        points.append(feature.feature_point.points[xk].y)
                    points=np.array(points).reshape(-1,1)
                    JM.initialJ(r1,r2,points)

                    object_flag=1
                    cos_flag=0 # cos term active flag
                    g = [] # equal constrains
                    lbg = []
                    ubg = []
                    for i in range(N+1):
                        # g.append(ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2))
                        # lbg.append(0)
                        # ubg.append(1) 
                        for j in range(8):
                            if j%3==0:
                                g.append(X[i, j])
                                lbg.append(15*1.5306122e-3)
                                ubg.append(1140*1.5306122e-3 )
                            if j%3==1:
                                g.append(X[i, j])
                                lbg.append(35* 1.5037594e-3)
                                ubg.append(450* 1.5037594e-3)
                        for j in range(8,12,1):
                            if j%2==0:
                                g.append(X[i, j])
                                lbg.append(20*1.5306122e-3)
                                ubg.append(1140*1.5306122e-3 )
                            else:
                                g.append(X[i, j])
                                lbg.append(20* 1.5037594e-3)
                                ubg.append(460* 1.5037594e-3)
                    
                    for i in range(N+1):
                        g.append(ca.norm_2(X[i,:2]-X[i,3:5]))
                        lbg.append(L*0.6)
                        ubg.append(L*0.8)


                if object_flag==1:  
                    
                    Kt=10000
                    ddp=1.4
                    beta_max=1.8
                    
                    if len(feature.feature_point.points)!=3:
                        x0[:6]=np.array([Robot.robotx[RID+0], Robot.roboty[RID+0],Robot.robotyaw[RID+0],Robot.robotx[RID+1], Robot.roboty[RID+1],Robot.robotyaw[RID+1]]).reshape(-1,1)
                    else:         
                        x0= [Robot.robotx[RID+0], Robot.roboty[RID+0],Robot.robotyaw[RID+0],Robot.robotx[RID+1], Robot.roboty[RID+1],Robot.robotyaw[RID+1]]
                        for xk in range(N_target):
                            x0.append(feature.feature_point.points[xk].x)
                            x0.append(feature.feature_point.points[xk].y)

                    x0=np.array(x0).reshape(-1,1)
                    x1=np.concatenate(x0)

                    x_center=x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12]
                    x_center=(x_center/(N_target+2)).reshape(1,-1)
                    if cos_flag==0 and np.linalg.norm(Target_circle[:2]-x_center)<400*1.5037594e-3:
                        cos_flag=1
                        # ktheta=0.05
                    cos_theta=np.dot(-x0[8:10]+Target_circle[:2],(x0[:2]+x0[3:5])/2-x0[8:10])/np.linalg.norm(-x0[8:10]+Target_circle[:2])/np.linalg.norm((x0[:2]+x0[3:5])/2-x0[8:10])
                    # get gredient
                    [dx1,dx2]=distence_to_x((x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12])/(2+N_target),xs,Qr,JM.J)
                    # Dcos=getdcos_to_qm(x0[:2],x0[3:5],x0[8:10],xs,J[2:4,:2],J[2:4,-2:])
                    [Dcosx1,Dcosx2]=getdcos_to_x(x0[:2],x0[3:5],x0[8:10],xs,JM.J[2:4,:2],JM.J[2:4,-2:])
                    ktheta=2*max(np.linalg.norm(dx1)/np.linalg.norm(Dcosx1),np.linalg.norm(dx2)/np.linalg.norm(Dcosx2))
                    # update enviroment information
                    # get the closest points of the obstacle
                    # closest_point=[]
                    # [min_distance,min_point]=distance_point_to_polygon(x0[:2].reshape(1,-1), polygon) # x1
                    # closest_point.append(min_point)
                    # [min_distance,min_point]=distance_point_to_polygon(x0[3:5].reshape(1,-1), polygon) #x2
                    # closest_point.append(min_point)
                    # [min_distance,min_point]=distance_point_to_polygon(((x0[:2]+x0[3:5])/2).reshape(1,-1), polygon) #xm
                    # closest_point.append(min_point)
                    # ktheta=1.5*np.linalg.norm(Dx)/(np.linalg.norm(Dcosx1)+np.linalg.norm(Dcosx2))/2
                    error_x.append( Target_circle[0]-x_center[0][0])
                    error_y.append( Target_circle[1]-x_center[0][1])
                    error_d.append(np.linalg.norm(Target_circle[:2]-x_center[0]))
                    error_theta.append(math.acos(np.dot(Target_circle[:2]-x1[8:10],(x1[:2]+x1[3:5])/2-x1[8:10])/np.linalg.norm(Target_circle[:2]-x1[8:10])/np.linalg.norm((x1[:2]+x1[3:5])/2-x1[8:10])))
                    # Errorplot.on_running(model_error_x,model_error_y)
                    if x_next is not None:
                        error_model.append(np.linalg.norm(x_next[6:]-x0[6:]))
                        error_dmodel_update=[error_model]
                        Jerror.on_running(error_dmodel_update)
                    
                    error_d_update=[error_d]
                    Disterror.on_running(error_d_update)
                    error_theta_update=[error_theta]
                    Thetaerror.on_running(error_theta_update)
                    # if alpha.id!=-1:                           
                    #     xm=(x0[:2]+x0[3:5])/2
                    rho=[]
                    for j in range(N_target+1):
                        if j <1:
                            rho.append(x0[0:2])
                        else:
                            rho.append(x0[2*j+4:2*j+6])
                    rho.append(x0[3:5])   
                    #     Tt = np.linspace(0, 1, N_target+2)
                    #     B = np.linalg.inv(bmatrix(Tt,N_target+1))
                    #     rho1=np.zeros((2,1))
                    #     for j in range(N_target+2):
                    #         rho1=rho1+B[alpha.id,j]*rho[j]
                    #     rho2=np.zeros((2,1))
                    #     for j in range(N_target+2):
                    #         rho2=rho2+B[alpha.id+1,j]*rho[j]
                    #     M=np.transpose(np.concatenate((rho1-xm,rho2-xm), axis=1))
                    #     beta=[]
                    #     for j in range(Obstacle.points.shape[0]):
                    #         beta.append(np.matmul(np.matmul(np.ones((1,2)),np.linalg.inv(M)),Obstacle.points[j,:].reshape(2,1)-xm))
                    #         if j==0:
                    #             beta_min=beta[0]
                    #         else:
                    #             if beta[j]<beta_min:
                    #                 beta_min=beta[j]
                    #     if beta_min>beta_max:
                    #         F=0
                    #     elif beta_min>1:
                    #         F=1-np.cos(np.pi*(beta_max-beta_min)/(beta_max-1))
                    #     else:
                    #         F=1
                    
                    for j in range(len(Hepyplane.ID)):
                        W=np.linalg.norm(Hepyplane.w)*1.5037594e-3
                        d=(np.matmul(Hepyplane.w,rho[Hepyplane.ID[j]])+(Hepyplane.b+1)*1.5037594e-3)/W
                        print(d)

                    if np.linalg.norm(x_center[0]-Target_circle[:2])>r_object* 1.5037594e-3*0.6:
                        enclose_flag=False
                        enclose_flag_pub.publish(enclose_flag)
                        obj = 0 
                        start_time = time.time()
                        for i in range(N):
                            obj = obj +  ca.mtimes([U[i, :], R, U[i, :].T])+ca.mtimes([(X[i,:2]+X[i,3:5]+X[i,6:8]+X[i,8:10]+X[i,10:12])/(N_target+2)-P[-2:].T,Qr,((X[i,:2]+X[i,3:5]+X[i,6:8]+X[i,8:10]+X[i,10:12])/(N_target+2)-P[-2:].T).T])+\
                                Kt*ca.if_else(ca.norm_2(X[i,:2].T-Target_circle[:2])<Target_circle[2]*(ddp+0.5),(ca.norm_2(X[i,:2].T-Target_circle[:2])-Target_circle[2]*(ddp+0.5))**2,0)+\
                                Kt*ca.if_else(ca.norm_2(X[i,3:5].T-Target_circle[:2])<Target_circle[2]*(ddp+0.5),(ca.norm_2(X[i,3:5].T-Target_circle[:2])-Target_circle[2]*(ddp+0.5))**2,0)
                            if cos_flag==1:
                                obj=obj-ktheta*ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2)
                            for f_tube in range(N_target):
                                obj=obj+Kt*ca.if_else(ca.norm_2(X[i,6+2*f_tube:8+2*f_tube].T-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2(X[i,6+2*f_tube:8+2*f_tube].T-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                            obj=obj+Kt*ca.if_else(ca.norm_2((X[i,3:5].T+X[i,10:].T)/2-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2((X[i,3:5].T+X[i,10:].T)/2-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                            obj=obj+Kt*ca.if_else(ca.norm_2((X[i,:2].T+X[i,6:8].T)/2-Target_circle[:2])<Target_circle[2]*ddp,(ca.norm_2((X[i,:2].T+X[i,6:8].T)/2-Target_circle[:2])-Target_circle[2]*ddp)**2,0)
                            # beta scaled
                            # for j in range(3):
                            #     if j<2:
                            #         d=ca.norm_2(X[i,3*j:3*j+2]-closest_point[j].reshape(1,-1))
                            #     else:
                            #         Xm=(X[i,:2]+X[i,3:5])/2
                            #         d=ca.norm_2(Xm-closest_point[j].reshape(1,-1))
                            #     Fo=ca.if_else(d<dmin,1+ca.cos(ca.pi*d/dmin),0)
                            #     obj=obj+Fo
                            # if alpha.id!=-1:                           
                            #     Xm=(X[i,:2]+X[i,3:5])/2
                            q=[]
                            for j in range(N_target+1):
                                if j<1:
                                    q.append(X[i,0:2])
                                else:
                                    q.append(X[i,2*j+4:2*j+6])
                            q.append(X[i,3:5])
                         

                            ## obstacle avoidance 
                            dx=[]
                            Fo=[]
                            for j in range(N_target+2):
                            # for j in range(len(Hepyplane.ID)):
                                W=np.linalg.norm(Hepyplane.w)*1.5037594e-3
                                # d=(ca.mtimes(Hepyplane.w,ca.transpose(q[Hepyplane.ID[j]]))+(Hepyplane.b+1)*1.5037594e-3)/W
                                # dx.append((ca.mtimes(Hepyplane.w,ca.transpose(q[Hepyplane.ID[j]]))+(Hepyplane.b+1)*1.5037594e-3)/W)
                                dx.append((ca.mtimes(Hepyplane.w,ca.transpose(q[j]))+(Hepyplane.b+1)*1.5037594e-3)/W)
                                if j==0 or j==N_target+1:
                                    dmin=60
                                else:
                                    dmin=35
                                Fo.append(ca.if_else(dx[j]<dmin,1+ca.cos(ca.pi*dx[j]/dmin),0))
                                obj=obj+5*Fo[j]

                        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
                        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
                        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
                        
                        ## set parameter
                        c_p = np.concatenate((x0, xs))
                        init_control = ca.reshape(u0, -1, 1)
                        t_ = time.time()
                        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)

                        elapsed_time = time.time() - start_time
                        print("Execution time: {:.2f} seconds".format(elapsed_time))
                        calculate_time.append(elapsed_time)
                        calculate_time_update=[calculate_time]
                        Cal_time.on_running(calculate_time_update)
                        u_sol = ca.reshape(res['x'],  N, n_controls*2) # one can only have this shape of the output
                        x_next, u0 = shift_movement(T, x0, u_sol, f,un,JM.J)
                        u_c.append(np.array(u_sol.full()))
                        xx.append(x0.tolist())
                        vel=v2w(np.array(u_sol.full()),un)
                        JM.unpdateJ(x0[:2],x0[3:5],x0[-2*N_target:])
                        [X,ff]=gene_ff(X,U,P,f,JM.J)
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
                        if key.state is False and save_flag==0:
                            save_flag=1
                            Disterror.save_plot('distanceerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','distance'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                            Thetaerror.save_plot('thetaerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','theta'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                            Jerror.save_plot('error_model'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','modelerror'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                            Cal_time.save_plot('Cal_time'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Cal_time'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                    else:
                        # save data
                        # Errorplot.save_plot('error.png','error.csv')
                        Disterror.save_plot('distanceerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','distance'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                        Thetaerror.save_plot('thetaerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','theta'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                        Jerror.save_plot('error_model'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','modelerror'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                        Cal_time.save_plot('Cal_time'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','Cal_time'+str(QRID)+'target'+str(Targe_id.ID)+'.csv')
                        ran_vel=np.zeros((1,4))
                        vel_msg = Float64MultiArray(data=ran_vel[0])
                        rospy.loginfo(vel_msg)
                        pub.publish(vel_msg)
                        d = rospy.Duration(0.005)
                        rospy.sleep(d)
                        object_flag=0
                        enclose_flag=True
                        enclose_flag_pub.publish(enclose_flag)
    # except KeyboardInterrupt:
    #     print('exit')
    #     Disterror.save_plot('distanceerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','distance'+str(QRID)+str(Targe_id.ID)+'.csv')
    #     Thetaerror.save_plot('thetaerror'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','theta'+str(QRID)+str(Targe_id.ID)+'.csv')
    #     Jerror.save_plot('error_model'+str(QRID)+'Target'+str(Targe_id.ID)+'.png','modelerror'+str(QRID)+str(Targe_id.ID)+'.csv')
    
    except rospy.ROSInterruptException:
        pass

    # signal.signal(signal.SIGINT,exit)
    # signal.signal(signal.SIGTERM,exit)
