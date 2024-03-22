from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
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
from scipy.special import comb
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

import cvxopt.solvers

# from dlodynamics import dlodynamics
# from Dynamicupdateplot import DynamicUpdate
from pulp import *
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
            self.points[int(ID-2),0]=msg.robot_pose_array[i].position.x 
            self.points[int(ID-2),1]=msg.robot_pose_array[i].position.y 
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
            pose.position.x= pose.position.x
            pose.position.y= pose.position.y
            self.feature_point.points.append(pose.position)
        if len(msg.poses)%2==1: # odd
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses)+1)/2)].x* 1.5037594e-3 #to m
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses)+1)/2)].y* 1.5306122e-3 
        else:
            self.middlepoint.x=self.feature_point.points[int((len(msg.poses))/2)].x* 1.5037594e-3
            self.middlepoint.y=self.feature_point.points[int((len(msg.poses))/2)].y* 1.5306122e-3 
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.poses)
def get_control_points(points,degree):
    xpoints=[]
    ypoints=[]
    for i in range(len(points)):
        xpoints.append(points[i,0])
        ypoints.append(points[i,1])
    # Get the Bezier parameters based on a degree.
    data = get_bezier_parameters(xpoints, ypoints, degree)
    # x_val = [x[0] for x in data]
    # y_val = [x[1] for x in data]
    return data

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
            self.robotx[int(ID-1)]=msg.robot_pose_array[i].position.x
            self.roboty[int(ID-1)]=msg.robot_pose_array[i].position.y
            self.robotID[int(ID-1)]=msg.robot_pose_array[i].ID.data
            self.robotyaw[int(ID-1)]=msg.robot_pose_array[i].yaw
        self.flag=1

class Targe_ID:
    def __init__(self,QRID):
        self.flag=False
        self.transport_flag=False
        self.enclose_flag=False
        self.ID=0
        self.sub=rospy.Subscriber('goflag'+str(QRID),Bool,self.flag_callback,queue_size=10)
        self.subID=rospy.Subscriber('TargetID'+str(QRID),Int8,self.ID_callback,queue_size=10)
        self.transport_sub=rospy.Subscriber('transportflag'+str(QRID),Bool,self.transport_flag_callback,queue_size=10)
        self.enclose_sub=rospy.Subscriber('encloseflag'+str(QRID),Bool,self.enclose_flag_callback,queue_size=10)
    def flag_callback(self,msg):
        self.flag=msg.data
    def transport_flag_callback(self,msg):
        self.transport_flag=msg.data
    def ID_callback(self,msg):
        self.ID=int(msg.data)
    def enclose_flag_callback(self,msg):
        self.enclose_flag=msg.data



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


def get_dataset(rho,O):
    X1 = np.array(rho)
    y1 = np.ones(len(X1))
    X2 = np.array(O)
    y2 = np.ones(len(X2)) * -1
    X, y = get_dataset_for(X1, y1, X2, y2)
    return X, y

def get_dataset_for(X1, y1, X2, y2):
    X = np.vstack((X1, X2))
    y = np.hstack((y1, y2))
    return X, y


# computing 'w'
def compute_w(multipliers, X, y):
    return np.sum(multipliers[i] * y[i] * X[i] for i in range(len(y))) 


# compute 'b'
def compute_b(w, X, y):
    return np.sum([y[i] - np.dot(w, X[i]) for i in range(len(X))])/len(X)

def find_decision_boundary(w,b, x1min, x1max, x2min, x2max, diff):
    x1 = np.linspace(x1min, x1max, 1000)
    x2 = np.linspace(x2min, x2max, 1000)

    cordinates = [(x, y) for x in x1 for y in x2]
    x_cord, y_cord = zip(*cordinates)
    c_val = pd.DataFrame({'x1':x_cord, 'x2':y_cord})
    c_val['cval'] = c_val['x1']*w[0]+c_val['x2']*w[1]+b+1

    decision = c_val[np.abs(c_val['cval']) < diff]
    
    return decision.x1, decision.x2

def get_wb(X,y):
    
    m = X.shape[0] 

    # Gram matrix - The matrix of all possible inner products of X. 
    K = np.array([np.dot(X[i], X[j])
                for i in range(m)
                for j in range(m)]).reshape((m, m)) 
    P = cvxopt.matrix(np.outer(y, y) * K) 
    q = cvxopt.matrix(-1 * np.ones(m)) 

    # let alpha be 1 for all obervations
    # Equality constraints 
    A = cvxopt.matrix(y, (1, m)) # create 1*m matrix of y 
    b = cvxopt.matrix(0.0) 

    # Inequality constraints 
    G = cvxopt.matrix(np.diag(-1 * np.ones(m))) 
    h = cvxopt.matrix(np.zeros(m)) 

    # Solve the problem 
    solution = cvxopt.solvers.qp(P, q, G, h, A, b) # return # dictionary which contains iterators

    # Lagrange multipliers 
    multipliers = np.ravel(solution['x']) 
    # sorted(multipliers)

    # Support vectors have positive multipliers. 
    has_positive_multiplier = multipliers > 1e-07 
    sv_multipliers = multipliers[has_positive_multiplier]

    support_vectors_id=  np.where(multipliers > 1e-07 )[0]

    support_vectors = X[has_positive_multiplier] 
    support_vectors_y = y[has_positive_multiplier] 
    
    rho_id=np.where(y>0)[0]
    Id = np.intersect1d(support_vectors_id,rho_id)
    # w = compute_w(multipliers, X, y) 
    w_from_sv = compute_w(sv_multipliers, support_vectors, support_vectors_y)
    
    # print(w)          
    # print(w_from_sv)  

    margin = 2 / np.linalg.norm(w_from_sv)
    print(margin)

    b = compute_b(w_from_sv, support_vectors, support_vectors_y)  

    return w_from_sv,b,Id



if __name__ == '__main__':
    try:
        rospy.init_node('Obstacle_avoidence')
        pointname=rospy.get_param('~feature')
        QRID=rospy.get_param('~QRID')
        intersectionP_pub=rospy.Publisher('InterP'+str(QRID),Float32MultiArray,queue_size=10)
        Vertex_pub=rospy.Publisher('Vertex'+str(QRID),Float32MultiArray,queue_size=10)
        IP_flag_pub=rospy.Publisher('Flag_side'+str(QRID),Float32MultiArray,queue_size=10)
        control_points_pub=rospy.Publisher('controlpoints'+str(QRID),Float32MultiArray,queue_size=10)
        alphaID_pub=rospy.Publisher('alphaID'+str(QRID),Int8,queue_size=10)
        hypeplane_pub=rospy.Publisher('wb',Point32,queue_size=10)
        hypeplane_pull_pub=rospy.Publisher('wbpull',Point32,queue_size=10)
        Id_publisher = rospy.Publisher('IDarray', Int32MultiArray, queue_size=10)
        vel = [0]*2
        Robot = QRrobot()
        feature=Point_tube(pointname)
        Targe_id=Targe_ID(QRID)
        
        F=FLAG(QRID)
        r_object=32
        rate = rospy.Rate(10)
        object_flag=0
        Obstacle=Obstacle_QR()
        
        while not rospy.is_shutdown():
            if Robot.flag==1 and feature.middlepoint.x!=0 and Targe_id.transport_flag is False and F.enclose_flag is False and Obstacle.flag==1 :
                if len(feature.feature_point.points)!=3:
                    x0[:6]=np.array([Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]).reshape(-1,1)
                    feature_points[0]=[Robot.robotx[0], Robot.roboty[0]]
                    feature_points[-1]=[Robot.robotx[1], Robot.roboty[1]]
                    x0.append([])
                else:
                    feature_points=[[Robot.robotx[0], Robot.roboty[0]]]
                    x0= [Robot.robotx[0], Robot.roboty[0],Robot.robotyaw[0],Robot.robotx[1], Robot.roboty[1],Robot.robotyaw[1]]
                    for xk in range(N_target):
                        feature_points.append([feature.feature_point.points[xk].x,feature.feature_point.points[xk].y])
                        x0.append(feature.feature_point.points[xk].x)
                        x0.append(feature.feature_point.points[xk].y)
                    feature_points.append([Robot.robotx[1], Robot.roboty[1]])
                # get w b
                O=[]
                for j in range(len(Obstacle.points)):
                    O.append(Obstacle.points[j,:])
                X, y = get_dataset(feature_points,O)
                #publish hypeplane coie
                [w,b,Id]=get_wb(X,y)
                wb=Point32()
                wb.x=w[0]
                wb.y=w[1]
                wb.z=b
                hypeplane_pub.publish(wb)
                # pubish closet points id
                array_msg = Int32MultiArray()
                array_msg.data = Id
                Id_publisher.publish(array_msg)

                # get control points
                control_points=get_control_points(np.array(feature_points),N_target+1)
                # x0=np.array(x0).reshape(-1,1)
                # x1=np.concatenate(x0)
                # x_center=x0[:2]+x0[3:5]+x0[8:10]+x0[6:8]+x0[10:12]
                # x_center=(x_center/(N_target+2)).reshape(1,-1)
                CP=np.zeros((2*(N_target+2),1))
                inter=np.zeros((4*(N_target+2),1))
                for i in range(N_target+2):
                    CP[2*i:2*(i+1),:]=np.array(control_points[i]).reshape(-1,1)
                iP=Float32MultiArray(data=CP)
                control_points_pub.publish(iP)
         
                # calculate the alpha of hypeplane
                # alpha=[]
                # Xm=(np.array(feature_points[0])+np.array(feature_points[N_target+1]))/2
                # m=np.zeros((2,2))
                # for i in range(N_target+1):
                #     m[0,:]=(np.array(feature_points[i])-Xm).reshape(1,-1)
                #     m[1,:]=(np.array(feature_points[i+1])-Xm).reshape(1,-1)
                #     alpha.append(np.dot(np.linalg.pinv(m),np.ones((2,1))))
                # beta_min=float('inf')
                # flag_min=-1
                # for i in range(N_target+1):                      
                #     model = pulp.LpProblem('linear_programming', LpMaximize)
                #     # get solver
                #     solver = getSolver('PULP_CBC_CMD')
                #     # declare decision variables
                #     beta = LpVariable('x', lowBound = 1, cat = 'continuous')
                #     # Set the objective function
                #     model += beta
                #     for j in range(len(feature_points)):
                #         model+= beta >=np.dot(np.transpose(alpha[i]),(np.array(feature_points[j])-Xm).reshape(-1,1)) 
                #     for j in range(Obstacle.points.shape[0]):
                #         model += beta <= np.dot(np.transpose(alpha[i]),(np.array(Obstacle.points[j,:])-Xm).reshape(-1,1))
                #     # solve 
                #     results = model.solve(solver=solver)
                #     if LpStatus[results] == 'Optimal': 
                #         # print('The solution is optimal.')
                #         # print(f'Solution: beta* = {value(beta)}')
                #         if value(beta)<beta_min:
                #             beta_min=value(beta)
                #             flag_min=i

                # alphaID=Int8()
                # if beta_min>2:
                #     alphaID.data=-1
                # else:
                # # alphaID.data=flag_min
                # alphaID_pub.publish(alphaID)
            # for pulling progress
            if Robot.flag==1 and Targe_id.enclose_flag is True and Targe_id.transport_flag is False:
                x0= [[Robot.robotx[0], Robot.roboty[0]]]
                x0.append([Robot.robotx[1], Robot.roboty[1]])
                IDi=int(Targe_id.ID-1)
                x0.append([Robot.robotx[IDi],Robot.roboty[IDi]])
                O=[]
                for j in range(len(Obstacle.points)):
                    O.append(Obstacle.points[j,:])
                X, y = get_dataset(x0,O)
                [w,b,Id]=get_wb(X,y)
                wb=Point32()
                wb.x=w[0]
                wb.y=w[1]
                wb.z=b
                hypeplane_pull_pub.publish(wb)
                # pubish closet points id


    except rospy.ROSInterruptException:
        pass
