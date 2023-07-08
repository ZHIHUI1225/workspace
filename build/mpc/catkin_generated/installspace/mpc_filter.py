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
# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=5 # feature points number

class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.flag=0
        self.sub = rospy.Subscriber('/filter_points', PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        self.feature_point=PointCloud()
        for pose in msg.poses:
            pose.position.x= pose.position.x* 1.5037594e-3
            pose.position.y= pose.position.y* 1.5306122e-3 
            self.feature_point.points.append(pose.position)
        self.flag=1


class QRrobot:
    def __init__(self,N):
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
            ID=int(msg.robot_pose_array[i].ID.data-10)
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

if __name__ == '__main__':
    try:
        rospy.init_node('mpc_mode')

        pub = rospy.Publisher('anglevelocity', Float64MultiArray, queue_size=10)
        vel = [0]*2

        Robot = QRrobot(2)
        feature=Point_tube()
        # Frame=frame_image()
        T = 0.1# sampling time [s]
        N = 40 # prediction horizon
        un= 2# control step
        v_max = 0.02
        omega_max = 0.5
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

        P = ca.SX.sym('P', n_states*2+2*N_target+2*N_target)#initial states +target states 2 of s


        ### define
        X[0,:] = P[:n_states*2+2*N_target] # initial condiction
        #J=np.array([[0.5,0,0.5,0],[-0,0.5,-0,0.5]])
        J=np.array([[ 0.9526469,  -0.1075197 , -0.77689017, -0.39523037],\
                    [ 0.18947883 ,-0.14580687 ,-0.16369805 , 0.01455448],\
                    [ 0.92897864 ,-0.15695447 ,-0.81950744 ,-0.18188445],\
                    [ 0.40949242 ,-0.289551 ,  -0.32234588 ,-0.07496356],\
                    [ 0.71567424, -0.08485667 ,-0.57281997 ,-0.32470085],\
                    [ 0.33466186, -0.27229132 ,-0.31601518 , 0.10818207],\
                    [ 0.80538388 ,-0.10589923 ,-0.56712143 ,-0.58200715],\
                    [ 0.27275623 ,-0.28875897 ,-0.28146172 , 0.19194218],\
                    [ 0.7884849 , -0.02922406 ,-0.58436056 ,-0.52555243],\
                    [ 0.19457704, -0.36182125 ,-0.19460691 , 0.20159166]])

        ### define the relationship within the horizon
        for i in range(N):
            f_value = f(X[i, :n_states], U[i, :2])
            X[i+1, :n_states] = X[i, :n_states] + f_value*T
            f_value = f(X[i, n_states:2*n_states], U[i, 2:])
            X[i+1, n_states:2*n_states] = X[i, n_states:2*n_states] + f_value*T
            # J matrix
            X[i+1,-2*N_target:]=X[i,-2*N_target:]+ca.mtimes(J,ca.vertcat(X[i+1,:2].T-X[i,:2].T,X[i+1,3:5].T-X[i,3:5].T)).T

        ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

        Q = np.eye(2*N_target)
        R=0.00001*np.eye(4)
        #### cost function
        obj = 0 #### cost
        for i in range(N):
            #without angle error
            #obj = obj + ca.mtimes([X[i, -2:]-P[-2:].T, Q, (X[i, -2:]-P[-2:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])-0.03*ca.norm_2(X[i,:2]-X[i,3:-3])*ca.norm_2((X[i,:2]+X[i,3:-3])/2-X[i, -2:])
            obj = obj + ca.mtimes([X[i, -2*N_target:]-P[-2*N_target:].T, Q, (X[i, -2*N_target:]-P[-2*N_target:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])
        g = [] # equal constrains
        lbg = []
        ubg = []
        for i in range(N+1):
            for j in range(8):
                if j%3==0:
                    g.append(X[i, j])
                    lbg.append(30*1.5306122e-3)
                    ubg.append(1000*1.5306122e-3 )
                if j%3==1:
                    g.append(X[i, j])
                    lbg.append(30* 1.5037594e-3)
                    ubg.append(400* 1.5037594e-3)
        for i in range(N):
            g.append(ca.norm_2(X[i,:2]-X[i,3:5]))
            lbg.append(L*0.5)
            ubg.append(L*0.8)

        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
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
        xs = np.array([440* 1.5037594e-3, 200* 1.5306122e-3,420* 1.5037594e-3, 205* 1.5306122e-3,400* 1.5037594e-3, 210* 1.5306122e-3,380* 1.5037594e-3, 205* 1.5306122e-3,360* 1.5037594e-3, 200* 1.5306122e-3 ]).reshape(-1, 1) # final state
        # center=(int(xs[0]),int(xs[1]))
        # cv2.circle(Frame.image, center, 2, (255, 0, 255), -1)
        x_c = [] # contains for the history of the state
        u_c = []
        # t_c = [t0] # for the time
        xx = []

        u0 = np.array([0,0,0,0]*N).reshape(-1, 4)# np.ones((N, 2)) # controls
        while not rospy.is_shutdown():
            # print("time",time.time())
            if Robot.flag==1 and feature.flag==1:
                # x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],feature.middlepoint.x,feature.middlepoint.y]).reshape(-1, 1)# initial state
                x0= [Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]
                for xk in range(N_target):
                    x0.append(feature.feature_point.points[xk].x)
                    x0.append(feature.feature_point.points[xk].y)
                x0=np.array(x0).reshape(-1,1)
                # x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],Robot.robotx[2],Robot.roboty[2]]).reshape(-1, 1)# initial state
                if np.linalg.norm(x0[-2*N_target:]-xs)>0.03 :
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
                        # rospy.loginfo(vel_msg)
                        pub.publish(vel_msg)
                        d = rospy.Duration(T)
                        rospy.sleep(d)
                    ran_vel=np.zeros((1,4))
                    vel_msg = Float64MultiArray(data=ran_vel[0])
                    # rospy.loginfo(vel_msg)
                    pub.publish(vel_msg)
                    d = rospy.Duration(0.1)
                    rospy.sleep(d)

    except rospy.ROSInterruptException:
        pass
