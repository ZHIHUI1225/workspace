#!/usr/bin/env python3
# use the Aruco code  detect the tube feature points
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
import matplotlib.pyplot as plt
import csv
from std_msgs.msg import Float64MultiArray
import time
from scipy.optimize import fsolve
from scipy.special import ellipe
from cv_bridge import CvBridge
import scipy.optimize as opt
import datetime
import random

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=1 # feature points number
plt.ion()

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



class Jmatrix():
    def __init__(self,P:int):
        self.J=np.array([[0.5,0,0.5,0],[0,0.5,0,0.5]])
        self.p1=np.array((2,1))
        self.p2=np.array((2,1))
        self.e=0.00003 #error
        self.j=0
        self.points=np.array((P,1))
        self.datanumber=10 # the number of save date
        self.xdata=np.zeros((self.datanumber,4))
        self.ydata=np.zeros((self.datanumber,P))
    
    def initialJ(self,r1,r2,points):
        self.p1=r1
        self.p2=r2
        self.points=points 

    def unpdateJ(self,p1,p2,xtn):
        gamma=0.000005
        deltap=np.concatenate(((p1-self.p1).reshape(1,2),(p2-self.p2).reshape(1,2)), axis=1)
        self.p1=p1
        self.p2=p2
        # self.Tube.update(p1,p2)
        # # the change of tube
        # xtn=np.array(self.Tube.get_points(self.n_tube)).ravel().reshape(2*self.n_tube,1)
        #delta s
        delta_t=xtn-self.points
        self.points=xtn
        for i in range(len(self.points)):
            self.j=LA.norm(np.dot(deltap.reshape(1,4),self.J[i,:].reshape(4,1))-delta_t[i])**2/2+LA.norm(np.dot(self.xdata,self.J[i,:].reshape(4,1))-self.ydata[:,i].reshape(self.datanumber,1))**2/2
            if self.j>self.e:
                qn=self.J[i,:].reshape(4,1)\
                -gamma*(np.dot(self.xdata.T,np.dot(self.xdata,self.J[i,:].reshape(4,1))-self.ydata[:,i].reshape(self.datanumber,1))+np.dot(deltap.reshape(4,1),np.dot(deltap.reshape(1,4),self.J[i,:].reshape(4,1))-delta_t[i]).reshape(4,1))
                self.J[i,:]=qn.reshape(1,4)
        # update R
        self.xdata=np.vstack((self.xdata[len(deltap):,:],deltap))
         #update delta
        self.ydata=np.vstack((self.ydata[len(delta_t.T):,:],delta_t.T))


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

def carotationR(x0):  
    theta=ca.atan2(x0[4]-x0[1],x0[3]-x0[0])
    R=ca.horzcat(ca.vertcat(ca.cos(theta),-ca.sin(theta)), ca.vertcat(ca.sin(theta),ca.cos(theta)))
    O=x0[6:8].T
    return [R,O]

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
        pub = rospy.Publisher('anglevelocity', Float64MultiArray, queue_size=10)
        Targetzone_pub=rospy.Publisher('Targetzone',Point32,queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        model_errorplot=PlotUpdate(1)
        JM=Jmatrix(P=N_target*2)
        # Frame=frame_image()
        T = 0.15# sampling time [s]
        N = 40 # prediction horizon
        un= 5 # control step
        v_max = 0.015
        omega_max = 0.5
        rate = rospy.Rate(30)
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

        X = ca.SX.sym('X', (N+1), n_states*2+2) # x1,y1,theta1,x2,y1,thate2,xs,ys
        Xnew=ca.SX.sym('Xnew',N,2)
        P = ca.SX.sym('P', n_states*2+4)#initial states +target states 2 of object

        [X,ff]=gene_ff(X,U,P,f,JM.J)


        Q = 1*np.eye(2*N_target)
        R=0.0001*np.eye(4)
        Qr=0.05*np.eye(2)
        
        #### cost function
        obj = 0 #### cost
        for i in range(N):
            obj = obj +  ca.mtimes([U[i, :], R, U[i, :].T])+ca.mtimes([X[i,6:8]-P[-2:].T,Qr,(X[i,6:8]-P[-2:].T).T])
        lbx = []
        ubx = []
        for _ in range(N):
            lbx.append(0)
            ubx.append(v_max)
        for _ in range(N):
            lbx.append(-omega_max)
            ubx.append(omega_max)
        for _ in range(N):
            lbx.append(0)
            ubx.append(v_max)
        for _ in range(N):
            lbx.append(-omega_max)
            ubx.append(omega_max)

        x_c = [] # contains for the history of the state
        u_c = []
        # t_c = [t0] # for the time
        xx = []
        x_next=None
        model_error_x=[]
        model_error_y=[]
        u0 = np.array([0,0,0,0]*N).reshape(-1, 4)# np.ones((N, 2)) # controls
        object_flag=0
        r_object=35
        flag=0
        while not rospy.is_shutdown():
            if Robot.flag==1 :
                # object pick hard constraint
                if object_flag==0 and Robot.robotx[5]!=0:
                    Target_circle=np.array([500*1.5037594e-3, 200*1.5306122e-3, r_object* 1.5037594e-3])
                    xs=[]
                    for i in range(N_target):
                        xs.append(Target_circle[:2])
                    xs=np.array(xs).reshape(-1,1)
                # hard constraint
                    g = [] # equal constrains
                    lbg = []
                    ubg = []
                    for i in range(N+1):
                        for j in range(8):
                            if j%3==0:
                                g.append(X[i, j])
                                lbg.append(30*1.5306122e-3)
                                ubg.append(1200*1.5306122e-3 )
                                # ubg.append(1100*1.5306122e-3 )
                            if j%3==1:
                                g.append(X[i, j])
                                lbg.append(30* 1.5037594e-3)
                                ubg.append(450* 1.5037594e-3)
                    for i in range(N+1):
                        g.append(ca.norm_2(X[i,:2]-X[i,3:5]))
                        lbg.append(L*0.4)
                        ubg.append(L*0.8)
                    for i in range(N):
                        [M,O]=carotationR(X[i,:])
                        b=ca.mtimes(M,X[i,3:5].T)-ca.mtimes(M,O)
                        # b=ca.mtimes(M,X[i,3+n_tube:n_tube+5].T)-ca.mtimes(M,O)
                        Kb=ca.fabs(b[1]/b[0])
                        a=ca.mtimes(M,X[i,:2].T)-ca.mtimes(M,O)
                        # b=ca.mtimes(M,X[i,3+n_tube:n_tube+5].T)-ca.mtimes(M,O)
                        Ka=ca.fabs(a[1]/a[0])
                        Xnew[i,:]=(ca.mtimes(M,(X[i+1,6:8]).T)).T-ca.mtimes(M,O).T
                        # g.append(ca.fabs(Xnew[i,0])-(Xnew[i,1]/Kb))
                        # lbg.append(-100)
                        # ubg.append(0)
                        # g.append(ca.fabs(Xnew[i,0])-(Xnew[i,1]/Ka))
                        # lbg.append(-100)
                        # ubg.append(0)
                        g.append(Xnew[i,0])
                        lbg.append(-1)
                        ubg.append(1)
                        g.append(Xnew[i,1])
                        lbg.append(0)
                        ubg.append(1)

                    r1=np.array([[Robot.robotx[0]],[Robot.roboty[0]]])
                    r2=np.array([[Robot.robotx[1]],[Robot.roboty[1]]])
                    points=[]
                    for xk in range(N_target):
                            points.append(Robot.robotx[5])
                            points.append(Robot.roboty[5])
                    points=np.array(points).reshape(-1,1)
                    JM.initialJ(r1,r2,points)
                    object_flag=1
                    nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
                    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
                    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

                if object_flag==1:
                    # Create Point32 message
                    point = Point32()
                    point.x = Target_circle[0]/1.5037594e-3
                    point.y = Target_circle[1]/1.5306122e-3
                    point.z = Target_circle[2]/1.5037594e-3
                    Targetzone_pub.publish(point)
                # x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],feature.middlepoint.x,feature.middlepoint.y]).reshape(-1, 1)# initial state
                    x0= [Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]
                    x0.append(Robot.robotx[5])
                    x0.append(Robot.roboty[5])
                    x0=np.array(x0).reshape(-1,1)
                    x1=np.concatenate(x0)
                    if x_next is not None:
                        model_error_x.append(x_next[6][0]-x0[6][0])
                        model_error_y.append(x_next[7][0]-x0[7][0])
                        model_errorplot.on_running(model_error_x,model_error_y)
                    # x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],Robot.robotx[2],Robot.roboty[2]]).reshape(-1, 1)# initial state
                    # ee=[np.linalg.norm(x0[-2:]-xs[-2:])]
                    
                    # for i in range(1,N_target,1):
                    #     ee.append(np.linalg.norm(x0[-2*i-2:-2*i]-xs[-2*i-2:-2*i]))
                    # x_center=x0[:2]+x0[3:5]
                    x_center=x0[6:8]
                    x_center=np.concatenate(x_center)
                    if np.linalg.norm(x_center-Target_circle[:2])>r_object* 1.5037594e-3:
                        ## set parameter
                        c_p = np.concatenate((x0, xs))
                        init_control = ca.reshape(u0, -1, 1)
                        t_ = time.time()
                        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
                        # index_t.append(time.time()- t_)
                        u_sol = ca.reshape(res['x'],  N, n_controls*2) # one can only have this shape of the output
                        # ff_value = ff(u_sol, c_p) # [n_states, N]
                        x_next, u0 = shift_movement(T, x0, u_sol, f,un,JM.J)
                        # x_c.append(ff_value)
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
                        d = rospy.Duration(0.05)
                        rospy.sleep(d)

    except rospy.ROSInterruptException:
        pass
