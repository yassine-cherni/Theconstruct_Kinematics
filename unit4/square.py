#! /usr/bin/env python

import rospy, math, numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(msg):
    global phi   
    position = msg.pose.pose.position
    (_, _, phi) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z, 
                                         msg.pose.pose.orientation.w])
    
rospy.init_node('absolute_motion', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)

def velocity2twist(dphi, dx, dy):
    R = np.array([[1, 0, 0],
                  [0,  np.cos(phi), np.sin(phi)],
                  [0, -np.sin(phi), np.cos(phi)]])
    v = np.array([dphi, dx, dy])
    v.shape = (3,1)
    twist = np.dot(R, v)
    wz, vx, vy = twist.flatten().tolist()
    return wz, vx, vy

def twist2wheels(wz, vx, vy):
    
    # half of the wheel base distance
    l = 0.500/2
    # the radius of the wheels
    r = 0.254/2
    # half of track width
    w = 0.548/2
    
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r
    twist = np.array([wz, vx, vy])
    twist.shape = (3,1)
    u = np.dot(H, twist)
    return u.flatten().tolist()
    
motions = [(1,0), (0,1), (-1,0), (0,-1)]

for (mx, my) in motions:
    dx = mx
    dy = my
    # constant angular velocity for turning at 30 degrees/second
    dphi = math.radians(30)
    # publish this motion for about 3 seconds = iterations (300) x sleep value of 0.01 
    iterations = 300
    for _ in range(iterations):
        wz, vx, vy = velocity2twist(dphi, dx, dy)
        u = twist2wheels(wz, vx, vy)
        msg = Float32MultiArray(data=u)
        pub.publish(msg)
        rospy.sleep(0.01)
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)