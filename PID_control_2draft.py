#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
import numpy as np
import matplotlib.pyplot as plt
import array 

current_time = 0
old_time = 0

evsum = 0
evold = 0

ewsum = 0
ewold = 0

k = 0

#Timing Law

T=2

def s(t): return -4*np.pi*(t/T)**3 + 6*np.pi*(t/T)**2
def s_dot(t): return (-12*np.pi*(t/T)**2 + 12*np.pi*t/T)/T

#Trajectory
r = np.pi;
def q_d(t): return np.array([r*np.cos(s(t)),r*np.sin(s(t)),r*np.sin(s(t))])
def q_d_dot(t): return np.array([-r*np.sin(s(t))*s_dot(t), r*np.cos(s(t))*s_dot(t),r*np.cos(s(t))*s_dot(t)])

q_1 = array.array('d', []) 
q_2 = array.array('d', []) 


def time(increment=1):
    global current_time
    current_time += increment
    return current_time

def pid_controllerv(y, yc, Ta=0, Ti=1.0, Td=1.0, Kp=1, u0=0.0, e0=0.0):
	
	global evsum
	global evold
        e = yc-y
	Ki = Kp/Ti
	Kd = Kp*Td
	evsum = evsum+e
	u = Kp*e+Ki*Ta*evsum+Kd*(e-evold)/Ta
	evold = e
        
	return u

def pid_controllerw(y, yc, Ta=0, Ti=1.0, Td=1.0, Kp=1, u0=0.0, e0=0.0):
	
	global ewsum
	global ewold
        e = yc-y
	Ki = Kp/Ti
	Kd = Kp*Td
	ewsum = ewsum+e
	u = Kp*e+Ki*Ta*ewsum+Kd*(e-ewold)/Ta
	ewold = e
        
	return u





def huskyOdomCallback(message,cargs):
    global current_time
    global old_time
    global k
    global q_act

    # Callback arguments 
    pub,msg = cargs

    q=np.array([3.0,-1.0, 1.0])
    q_dot=np.array([0.0,0.0, 0.0])
    
    pos = message.pose.pose
    pose = [pos.position.x, pos.position.y]

    wdesired = q_d(k)[0]/10 # angular velocity [rad/s] 
    vdesired = q_d(k)[1]/10 # Linear velocity  [m/s]

    time()

    meas_w = message.twist.twist.angular.z
    meas_v = message.twist.twist.linear.x
    q_1.append(meas_w);
    q_2.append(meas_v);
    msg.angular.z = pid_controllerw(meas_w, wdesired, Ta=(current_time-old_time), Ti=0.1, Td=0.0, Kp=0.01, u0=0.0, e0=meas_w-wdesired)
    msg.linear.x = pid_controllerv(meas_v, vdesired, Ta=(current_time-old_time), Ti=1, Td=0.5, Kp=0.1, u0=0.0, e0=meas_v-vdesired)

    pub.publish(msg)

    old_time = current_time

    print('huskyOdomCallback: meas_v=%4.3f,meas_w=%4.3f act_w=%4.3f, act_v=%4.3f, desired_v=%4.3f, desired_w=%4.3f, x=%4.10f, y=%4.10f k=%4.10f'%(meas_v,meas_w,msg.angular.z,msg.linear.x, vdesired, wdesired,pose[0],pose[1],k))
    
    k = k + 0.01
    if k >= 2:
	x_plot=np.linspace(0,2,200)
	x_enum=np.linspace(0,199,200)
	plt.subplot(211)
	plt.plot(x_enum,q_1, color="blue")
	plt.plot(x_enum,q_d(x_plot)[0]/10, color="red")
	plt.title('trajectory tracking angular velocity')

	plt.ylabel('rad/s')
	plt.grid(True)
	plt.subplot(212)
	plt.plot(x_enum,q_2, color="green")
	plt.plot(x_enum,q_d(x_plot)[1]/10, color="black")
	plt.title('trajectory tracking linear velocity')
	plt.xlabel('time (s)')
	plt.ylabel('m/s')
	plt.grid(True)
	plt.show()




########################################
# Main Script
# Initialize our node
rospy.init_node('nre_simhuskycontrol',anonymous=True)
    
# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
rospy.Subscriber('odometry/filtered',nav_msgs.msg.Odometry,huskyOdomCallback, 
                 (cmdpub,cmdmsg))

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
