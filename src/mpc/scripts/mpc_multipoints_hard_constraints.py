#!/usr/bin/env python3
# use the Aruco code  detect the tube feature points
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
import cv2
import scipy.optimize as opt
# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=3 # feature points number
# plt.ion()
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
        self.sub=rospy.Subscriber('goflag',Bool,self.flag_callback,queue_size=10)
        self.subID=rospy.Subscriber('TargetID',Int8,self.ID_callback,queue_size=10)
        self.transport_sub=rospy.Subscriber('transportflag',Bool,self.transport_flag_callback,queue_size=10)
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

class PlotUpdate():
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
            xdata=np.arange(0,len(error_x))
            self.lines1[i].set_xdata(xdata)
            self.lines1[i].set_ydata(error_x)
            ydata=np.arange(0,len(error_y))
            self.lines2[i].set_xdata(ydata)
            self.lines2[i].set_ydata(error_y)
        #Need both of these in order to rescale
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


if __name__ == '__main__':
    try:
        rospy.init_node('mpc_mode')
        pub = rospy.Publisher('anglevelocity', Float64MultiArray, queue_size=10)
        enclose_flag_pub=rospy.Publisher('encloseflag',Bool,queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        feature=Point_tube()
        Targe_id=Targe_ID()
        model_errorplot=PlotUpdate(1)
        # Frame=frame_image()
        T = 0.15# sampling time [s]
        N = 30 # prediction horizon
        un= 4 # control step
        v_max = 0.015
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
        R=0.0001*np.eye(4)
        Qr=0.002*np.eye(2)
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
            if Robot.flag==1 and feature.middlepoint.x!=0 and Targe_id.transport_flag is False and enclose_flag is False:
                # object pick hard constraint
                
                if object_flag==0 and Targe_id.ID!=0:
                    # deltax=(20+(44-Robot.robotx[5]/1.5037594e-3/25.65))*1.5037594e-3
                    # deltay=(7+(32-Robot.roboty[5]/1.5306122e-3/29.71))*1.5306122e-3
                    IDi=Targe_id.ID-1
                    Target_circle=np.array([Robot.robotx[IDi], Robot.roboty[IDi], r_object* 1.5037594e-3])
                    xs=[]
                    # for i in range(N_target+2):
                    xs.append(Target_circle[:2])
                    xs=np.array(xs).reshape(-1,1)
                    object_flag=1
                     #### cost function
                    obj = 0 #### cost
                    for i in range(int(N/6),N):
                        #without angle error
                        #obj = obj + ca.mtimes([X[i, -2:]-P[-2:].T, Q, (X[i, -2:]-P[-2:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])-0.03*ca.norm_2(X[i,:2]-X[i,3:-3])*ca.norm_2((X[i,:2]+X[i,3:-3])/2-X[i, -2:])
                        # obj = obj + ca.mtimes([X[i, -2*N_target:]-P[-2*N_target:].T, Q, (X[i, -2*N_target:]-P[-2*N_target:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])
                        # obj = obj + ca.mtimes([X[i, -2*N_target:]-P[-2*N_target:].T, Q, (X[i, -2*N_target:]-P[-2*N_target:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])\
                        #     +ca.mtimes([X[i,:2]-P[-2*N_target-4:-2*N_target-2].T,Qr,(X[i,:2]-P[-2*N_target-4:-2*N_target-2].T).T])+ca.mtimes([X[i,3:5]-P[-2*N_target-2:-2*N_target].T,Qr,(X[i,3:5]-P[-2*N_target-2:-2*N_target].T).T])
                        obj = obj +  ca.mtimes([U[i, :], R, U[i, :].T])+ca.mtimes([(X[i,6:8]+X[i,8:10]+X[i,10:12])/3-P[-2:].T,Qr,((X[i,6:8]+X[i,8:10]+X[i,10:12])/3-P[-2:].T).T])-\
                            1*ca.dot(-X[i,8:10]+Target_circle[:2].reshape(1,-1),(X[i,:2]+X[i,3:5])/2-X[i,8:10])/ca.norm_2(-X[i,8:10]+Target_circle[:2].reshape(1,-1))/ca.norm_2(-X[i,8:10]+(X[i,:2]+X[i,3:5])/2)
                    
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
                                lbg.append(20* 1.5037594e-3)
                                ubg.append(450* 1.5037594e-3)
                        for j in range(8,12,1):
                            if j%2==0:
                                g.append(X[i, j])
                                lbg.append(30*1.5306122e-3)
                                ubg.append(1100*1.5306122e-3 )
                            else:
                                g.append(X[i, j])
                                lbg.append(20* 1.5037594e-3)
                                ubg.append(450* 1.5037594e-3)
                    ddp=1.2
                    for i in range(N+1):
                        g.append(ca.norm_2(X[i,:2]-X[i,3:5]))
                        lbg.append(L*0.5)
                        ubg.append(L*0.8)
                        g.append(ca.norm_2(X[i,:2].T-Target_circle[:2]))
                        lbg.append(Target_circle[2]*2.1)
                        ubg.append(200)
                        g.append(ca.norm_2(X[i,3:5].T-Target_circle[:2]))
                        lbg.append(Target_circle[2]*2.1)
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
                                lbg.append(C[2]*2.1)
                                ubg.append(200)
                                g.append(ca.norm_2(X[i,3:5].T-C[:2]))
                                lbg.append(C[2]*2.1)
                                ubg.append(200)
                                g.append(ca.norm_2((X[i,6:8].T+X[i,10:12].T+X[i,8:10].T)/3-C[:2]))
                                lbg.append(C[2]*2.1)
                                ubg.append(200)
                                for j in range(N_target):
                                    g.append(ca.norm_2(X[i,6+2*j:8+2*j].T-C[:2]))
                                    lbg.append(C[2]*ddp)
                                    ubg.append(200)

                    nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
                    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
                    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)


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
                    # if x_next is not None:
                    #     model_error_x.append(x_next[8][0]-x0[8][0])
                    #     model_error_y.append(x_next[9][0]-x0[9][0])
                    #     model_errorplot.on_running(model_error_x,model_error_y)
                    # x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],Robot.robotx[2],Robot.roboty[2]]).reshape(-1, 1)# initial state
                    # ee=[np.linalg.norm(x0[-2:]-xs[-2:])]
                    
                    # for i in range(1,N_target,1):
                    #     ee.append(np.linalg.norm(x0[-2*i-2:-2*i]-xs[-2*i-2:-2*i]))
                    # x_center=x0[:2]+x0[3:5]
                    x_center=x0[6:8]
                    for i in range(1,N_target):
                        x_center=x_center+x0[6+2*i:8+2*i]
                    x_center=(x_center/(N_target)).reshape(1,-1)
                    # if ee[0]>r_object* 1.5037594e-3*1.3 or ee[1]>r_object* 1.5037594e-3*1.3 or ee[2]>r_object* 1.5037594e-3*1.3:
                    if np.linalg.norm(x_center[0]-Target_circle[:2])>r_object* 1.5037594e-3*0.9:
                        enclose_flag=False
                        enclose_flag_pub.publish(enclose_flag)
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
                        d = rospy.Duration(0.02)
                        rospy.sleep(d)
                    else:
                        object_flag=0
                        enclose_flag=True
                        enclose_flag_pub.publish(enclose_flag)


    except rospy.ROSInterruptException:
        pass
