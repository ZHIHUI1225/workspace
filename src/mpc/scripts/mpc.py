#!/usr/bin/env python3
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
# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate

wheel_base = 80e-3  # mm
wheel_diameter = 31e-3  # mm
L=300e-3 #the length of tube

class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.middlepoint=Point32()
        self.sub = rospy.Subscriber('/feature_points', PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        self.feature_point=PointCloud()
        for pose in msg.poses:
            self.feature_point.points.append(pose.position)
        if len(msg.poses)%2==1: # odd
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses)+1)/2)].x* 1.5037594e-3 #to m
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses)+1)/2)].y* 1.5306122e-3 
        else:
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses))/2)].x* 1.5037594e-3
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses))/2)].y* 1.5306122e-3 
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.poses)


class QRrobot:
    def __init__(self,image=None,x=None,y=None,xmax=None,ymax=None):
        self.robotx=[0.0]*2
        self.roboty=[0.0]*2
        self.robotyaw=[0.0]*2
        self.robotID=[0]*2
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
        w1[i,0]= (uLinear1 - uAngular1 * wheel_base / 2) * 2 / wheel_diameter
        w1[i,1]= (uLinear1 + uAngular1 * wheel_base / 2) * 2 / wheel_diameter
        w2[i,0]= (uLinear2 - uAngular2 * wheel_base / 2) * 2 / wheel_diameter
        w2[i,1]= (uLinear2 + uAngular2 * wheel_base / 2) * 2 / wheel_diameter       
        w[i,:]=[w1[i,0],w1[i,1],w2[i,0],w2[i,1]]
    return w
        
if __name__ == '__main__':
    try:
        rospy.init_node('tf_listener_node')

        # pub1 = rospy.Publisher('anglevelocity1', Twist, queue_size=10)
        # pub2 = rospy.Publisher('anglevelocity2', Twist, queue_size=10)
        # vel_msg1=Twist()
        # vel_msg1.angular.x = 0 #wl
        # vel_msg1.angular.y = 0 #wr
        # vel_msg1.angular.z=0 #ID
        # vel_msg2=Twist()
        # vel_msg2.angular.x = 0
        # vel_msg2.angular.y = 0
        # vel_msg2.angular.z=0 #ID

        pub = rospy.Publisher('anglevelocity', Float64MultiArray, queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        feature=Point_tube()

        T = 0.5# sampling time [s]
        N = 10 # prediction horizon
        un=3 # control step
        v_max = 1e-3
        omega_max = np.pi/6e-3
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
        rhs = ca.horzcat(v*ca.cos(theta), v*ca.sin(theta))
        rhs = ca.horzcat(rhs, omega)
        ## function
        f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        ## for MPC
        U = ca.SX.sym('U', N, n_controls*2)

        X = ca.SX.sym('X', (N+1), n_states*3-1) # x1,y1,theta1,x2,y1,thate2,xs,ys

        P = ca.SX.sym('P', n_states*3-1+2)#initial states +target states 2 of s


        ### define
        X[0,:] = P[:n_states*3-1] # initial condiction
        J=np.array([[0.5,0,0.5,0],[-0,0.5,-0,0.5]])
        ### define the relationship within the horizon
        for i in range(N):
            f_value = f(X[i, :n_states], U[i, :2])
            X[i+1, :n_states] = X[i, :n_states] + f_value*T
            f_value = f(X[i, n_states:-2], U[i, 2:])
            X[i+1, n_states:-2] = X[i, n_states:-2] + f_value*T
            # J matrix
            X[i+1,-2:]=X[i,-2:]+ca.mtimes(J,ca.vertcat(X[i+1,:2].T-X[i,:2].T,X[i+1,3:5].T-X[i,3:5].T)).T

        ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

        Q = np.eye(2)
        R=0.01*np.eye(4)
        #### cost function
        obj = 0 #### cost
        for i in range(N):
            #without angle error
            #obj = obj + ca.mtimes([X[i, -2:]-P[-2:].T, Q, (X[i, -2:]-P[-2:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])-0.03*ca.norm_2(X[i,:2]-X[i,3:-3])*ca.norm_2((X[i,:2]+X[i,3:-3])/2-X[i, -2:])
            obj = obj + ca.mtimes([X[i, -2:]-P[-2:].T, Q, (X[i, -2:]-P[-2:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])
        g = [] # equal constrains
        lbg = []
        ubg = []
        for i in range(N+1):
            for j in range(4):
                g.append(X[i, j])
                lbg.append(0)
                ubg.append(200)
        for i in range(N):
            g.append(ca.norm_2(X[i,:2]-X[i,3:-3]))
            lbg.append(L/5)
            ubg.append(L)

        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
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
        xs = np.array([500e-3, 160e-3]).reshape(-1, 1) # final state
        x_c = [] # contains for the history of the state
        u_c = []
        # t_c = [t0] # for the time
        xx = []

        u0 = np.array([0,0,0,0]*N).reshape(-1, 4)# np.ones((N, 2)) # controls
        while not rospy.is_shutdown():
            if Robot.flag==1 and feature.middlepoint.x!=0:
                x0 = np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1],feature.middlepoint.x,feature.middlepoint.y]).reshape(-1, 1)# initial state
                if np.linalg.norm(x0[-2:]-xs)>0.1 :
                    ## set parameter
                    c_p = np.concatenate((x0, xs))
                    init_control = ca.reshape(u0, -1, 1)
                    t_ = time.time()
                    res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
                    # index_t.append(time.time()- t_)
                    u_sol = ca.reshape(res['x'],  N, n_controls*2) # one can only have this shape of the output
                    ff_value = ff(u_sol, c_p) # [n_states, N]
                    x_c.append(ff_value)
                    u_c.append(np.array(u_sol.full()))
                    xx.append(x0.tolist())
                    vel=v2w(np.array(u_sol.full()),un)
                    for i in range(un):
                        vel_msg = Float64MultiArray(data=vel[i,:])
                        rospy.loginfo(vel_msg)
                        pub.publish(vel_msg)
                        d = rospy.Duration(T)
                        rospy.sleep(d)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
