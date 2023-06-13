#!/usr/bin/env python3
# use the Bezier curve fitting
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from robot_msg.msg import robot_pose_array
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
from scipy.special import comb
import numpy as np
import cv2
import math
import casadi as ca
import casadi.tools as ca_tools
import time
import matplotlib.pyplot as plt
import csv
from std_msgs.msg import Float64MultiArray
from scipy.optimize import fsolve
from scipy.special import ellipe
from cv_bridge import CvBridge
import cv2
from numpy import linalg as LA
import scipy.optimize as opt

wheel_base = 80e-3  # m
wheel_diameter = 31e-3  # m
l_center=11* 1.5037594e-3
L=200* 1.5037594e-3 #the length of tube
N_target=3 # feature points number
plt.ion()
def get_bezier_parameters(X, Y, degree=3):
    """ Least square qbezier fit using penrose pseudoinverse.
    """
    if degree < 1:
        raise ValueError('degree must be 1 or greater.')

    if len(X) != len(Y):
        raise ValueError('X and Y must be of the same length.')

    if len(X) < degree + 1:
        raise ValueError(f'There must be at least {degree + 1} points to '
                         f'determine the parameters of a degree {degree} curve. '
                         f'Got only {len(X)} points.')

    def bpoly(n, t, k):
        """ Bernstein polynomial when a = 0 and b = 1. """
        return t ** k * (1 - t) ** (n - k) * comb(n, k)
        #return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bmatrix(T):
        """ Bernstein matrix for Bézier curves. """
        return np.matrix([[bpoly(degree, t, k) for k in range(degree + 1)] for t in T])

    def least_square_fit(points, M):
        M_ = np.linalg.pinv(M)
        return M_ * points

    T = np.linspace(0, 1, len(X))
    M = bmatrix(T)
    points = np.array(list(zip(X, Y)))
    
    final = least_square_fit(points, M).tolist()
    final[0] = [X[0], Y[0]]
    final[len(final)-1] = [X[len(X)-1], Y[len(Y)-1]]
    return final

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=50):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals


class Point_tube:
    def __init__(self):
        self.feature_point=PointCloud()
        self.controlpoints=np.zeros((1,10))
        # self.sub = rospy.Subscriber('/line_without_QR', PointCloud, self.tube_callback,queue_size=10)
        self.sub = rospy.Subscriber('control_points',PoseArray, self.tube_callback,queue_size=10)
    def tube_callback(self, msg): # tube msg
        i=0
        for pose in msg.poses:
            self.controlpoints[0][i]=pose.position.x* 1.5037594e-3
            self.controlpoints[0][i+1]=pose.position.y* 1.5306122e-3
            i=i+2
        # self.feature_point=PointCloud()
        # xpoints=[]
        # ypoints=[]
        # points=[]
        # for point in msg.points:
        #     self.feature_point.points.append(point)
        #     xpoints.append(point.x* 1.5037594e-3)
        #     ypoints.append(point.y* 1.5306122e-3)
        # for i in range(len(xpoints)):
        #     points.append([xpoints[i],ypoints[i]])
        # data = get_bezier_parameters(xpoints, ypoints, degree=4)
        # self.controlpoints=np.array(data).reshape(-1,1)[2:-2]

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

def my_func(x):
    if x < 0:
        return 0
    else:
        return x
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
    def get_control_points(self,degree):
        P=self.getshape()
        points = []
        xpoints=P[0]
        ypoints=P[1]
        for i in range(len(xpoints)):
            points.append([xpoints[i],ypoints[i]])
        # Get the Bezier parameters based on a degree.
        data = get_bezier_parameters(xpoints, ypoints, degree=4)
        x_val = [x[0] for x in data]
        y_val = [x[1] for x in data]
        return data

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
        Robot = QRrobot(N_target+2)
        feature=Point_tube()
        model_errorplot=PlotUpdate(1)
        Tube=tubeshape(length=L*0.85,p1=np.array([500* 1.5037594e-3,300* 1.5306122e-3 ]),p2=np.array([650* 1.5037594e-3,300* 1.5306122e-3 ]))
        xt=np.array(Tube.get_control_points(4)).reshape(-1,1)
        xs=np.reshape(xt,(len(xt),1))
        xs=np.vstack((xs[:2],xs[-2:],xs[2:-2]))
        # Frame=frame_image()
        T = 0.15# sampling time [s]
        N = 30 # prediction horizon
        un= 3 # control step
        v_max = 0.03
        omega_max = 0.8
        rate = rospy.Rate(1)
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

        P = ca.SX.sym('P', n_states*2+2*N_target+2*N_target+4)#initial states +target states 2 of s


        ### define
        X[0,:] = P[:n_states*2+2*N_target] # initial condiction
        #J=np.array([[0.5,0,0.5,0],[-0,0.5,-0,0.5]])
        J=np.array([[-0.00375903 ,-0.20224102, -0.01303673,  0.41626111],\
                    [-0.03168586 , 0.52710869 ,-0.27831966 , 0.41884191],\
                    [-0.15997836 , 0.07438409,  0.03951559 , 0.35761971],\
                    [ 0.38790225,  0.56109363 ,-0.64424796 , 0.20801625],\
                    [-0.37726967, -0.29525868 , 0.30223423 , 0.75935363],\
                    [ 0.13854681 , 0.70619581 ,-0.47986188 , 0.27337375]])
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
        Qr=0*np.eye(2)
        #### cost function
        obj = 0 #### cost
        for i in range(N):
            #without angle error
            #obj = obj + ca.mtimes([X[i, -2:]-P[-2:].T, Q, (X[i, -2:]-P[-2:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])-0.03*ca.norm_2(X[i,:2]-X[i,3:-3])*ca.norm_2((X[i,:2]+X[i,3:-3])/2-X[i, -2:])
            # obj = obj + ca.mtimes([X[i, -2*N_target:]-P[-2*N_target:].T, Q, (X[i, -2*N_target:]-P[-2*N_target:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])
             obj = obj + ca.mtimes([X[i, -2*N_target:]-P[-2*N_target:].T, Q, (X[i, -2*N_target:]-P[-2*N_target:].T).T])+ ca.mtimes([U[i, :], R, U[i, :].T])\
                +ca.mtimes([X[i,:2]-P[-2*N_target-4:-2*N_target-2].T,Qr,(X[i,:2]-P[-2*N_target-4:-2*N_target-2].T).T])+ca.mtimes([X[i,3:5]-P[-2*N_target-2:-2*N_target].T,Qr,(X[i,3:5]-P[-2*N_target-2:-2*N_target].T).T])
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
            lbg.append(L*0.35)
            ubg.append(L*0.8)

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
        # xs = np.array([400* 1.5037594e-3, 180* 1.5306122e-3,360* 1.5037594e-3, 200* 1.5306122e-3,440* 1.5037594e-3, 200* 1.5306122e-3 ]).reshape(-1, 1) # final state
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
        while not rospy.is_shutdown():
            if Robot.flag==1 and feature.controlpoints[0][0]!=0:               
                x0= [[Robot.robotx[0]], [Robot.roboty[0]],[Robot.robotyaw[0]],[Robot.robotx[1]], [Robot.roboty[1]],[Robot.robotyaw[1]]]
                xt=feature.controlpoints[0]
                xt=np.reshape(xt,(len(xt),1))
                xt=np.vstack((xt[2:-2]))
                x0=np.vstack((x0,np.reshape(xt,(len(xt),1))))
                if x_next is not None:
                    model_error_x.append(x_next[8][0]-x0[8][0])
                    model_error_y.append(x_next[9][0]-x0[9][0])
                    model_errorplot.on_running(model_error_x,model_error_y)
                # if np.linalg.norm(x0[-2*N_target:]-xs[-2*N_target:])+np.linalg.norm(x0[:2]-xs[-2*N_target-4:-2*N_target-2])+np.linalg.norm(x0[3:5]-xs[-2*N_target-2:-2*N_target])>0.1 :
                if np.linalg.norm(x0[-2*N_target:]-xs[-2*N_target:])>0.06 :
                    ## set parameter
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
                    d = rospy.Duration(0.1)
                    rospy.sleep(d)

    except rospy.ROSInterruptException:
        pass
