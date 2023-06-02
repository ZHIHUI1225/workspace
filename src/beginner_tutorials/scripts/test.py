#import controllaw 
import Staticcontrol
import time
import numpy as np
import math
from scipy.optimize import fsolve
from scipy.special import ellipe

def  get_targets(Xc,Yc,thetac,curvature,length):

    def sin_fitiing(x):
        x0=float(x[0])
        a=float(x[1])
        l=length # length of the tube
        K=curvature # curvature of the peak point
        return [l-(2*math.sqrt(x0**2+a**2 *math.pi**2))/math.pi * ellipe(1/math.sqrt(1+x0**2/(a**2 *math.pi**2))),
        K-a*math.pi**2 /x0**2
    ]
    result = fsolve(sin_fitiing, [1,1])
    x0=result[0]
    a=result[1]
    # coordinate translation and rotation
    positionc=np.array([[Xc],[Yc]])
    M=np.array([[math.cos(thetac),math.sin(thetac)],[-math.sin(thetac),math.cos(thetac)]])
    X0=np.array([[x0],[0]])
    position0=positionc-np.dot(M,np.array([[x0/2],[a]]))
    positon_I=np.dot(M,X0)+position0
    # return the two end points n inertia coordinate
    return [position0,positon_I]
    

def kinematics(x,y,theta,v,w):
    R=1
    L=10
    sample_time=0.01
    M=np.array([[1/R,L/(2*R)],[1/R,-L/(2*R)]])
    W=np.dot(M,np.array([[v],[w]]))
    wr=float(W[0])
    wl=float(W[1])
    x_new=x+R/2*(wr+wl)*math.cos(theta)*sample_time
    y_new=y+R/2*(wr+wl)*math.sin(theta)*sample_time
    theta_new=theta+R/L*(wr-wl)*sample_time
    if theta_new>math.pi:
        theta_new=-math.pi
    elif theta_new<-math.pi:
        theta_new=math.pi
    return [x_new,y_new,theta_new]

def test_control(Kv, Kw, target_x, target_y,initial_x,initial_y,initial_theta):
    control = Staticcontrol.controllaw(Kv,Kw)
    control.Setx=target_x
    control.Sety=target_y
    control.setSampleTime(0.01)
    error_x=target_x-initial_x
    error_y=target_y-initial_y
    x=initial_x
    y=initial_y
    theta=initial_theta
    v=0
    w=0
    while np.sqrt(error_x**2+error_y**2)>=0.1:
        control.update(x,y,theta)
        output_v = control.v_output
        output_w = control.w_output
        # last_output_v +=output_v # delta v w
        # last_output_w +=output_w
        # v=last_output_v
        # w=last_output_w
        v=output_v
        w=output_w
        [x_new,y_new,theta_new]=kinematics(x,y,theta,v,w)
        x=x_new
        y=y_new
        theta=theta_new
        error_x=target_x-x
        error_y=target_y-y
        #print(error_x,error_y,v,w,theta)
        time.sleep(0.01)
    return [x,y,theta]


Xc=4
Yc=9
thetac=0
curvature=1
length=10
target=get_targets(Xc,Yc,thetac,curvature,length)
print(target)
result=test_control(10,10,target[0][0],target[0][1],0.0,0.0,0.0)
print(np.array(result[0:2])-(np.array(target[0])).T[0])

result=test_control(10,10,target[1][0],target[1][1],1.0,1.0,0.0)
print(np.array(result[0:2])-(np.array(target[1])).T[0])