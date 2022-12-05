#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Given any planer twist the corresponding speed of wheels u can be obtained


def odom_callback(msg):
    global phi
    position = msg.pose.pose.position
    (_, _, phi) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w])


def twist2wheels(vx, vy, wz):
    l = 0.500/2
    w = 0.548/2
    r = 0.254/2
    H = np.array([[1, -1, -w-l],
                  [1, 1, w+l],
                  [1, 1, -w-l],
                  [1, -1, w+l]]) / r
    twist = np.array([vx, vy, wz])
    twist.shape = (3, 1)
    u = np.dot(H, twist) # fl fr bl br
    return u.flatten().tolist()


def velocity2twist(dx, dy, dphi):
    H = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi), np.cos(phi), 0],
                  [0, 0, 1]])
    velocity = np.array([dx, dy, dphi])
    velocity.shape = (3, 1)
    twist = np.dot(H, velocity)
    vx, vy, wz = twist.flatten().tolist()
    return vx, vy, wz


rospy.init_node('make_turn', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)

# u = twist2wheels(wz=1.5708, vx=1, vy=0)
# msg = Float32MultiArray(data=u)
# pub.publish(msg)
# rospy.sleep(1)
for _ in range(100):
    vx, vy, wz = velocity2twist(dx=1, dy=0, dphi=0)
    print(vx, vy, wz)
    u = twist2wheels(vx, vy, wz)
    msg = Float32MultiArray(data=u)
    pub.publish(msg)
    rospy.sleep(0.01)

stop = [0, 0, 0, 0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)
