#! /usr/bin/env python3
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

rospy.init_node('kinematic_controller', anonymous=True)


class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        rospy.sleep(0.1)

    def velocity2twist(self, dx, dy, dtheta):
        H = np.array([[np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi), np.cos(phi), 0],
                    [0, 0, 1]])
        velocity = np.array([dx, dy, dtheta])
        velocity.shape = (3, 1)
        twist = np.dot(H, velocity)
        vx, vy, wz = twist.flatten().tolist()
        return vx, vy, wz

    def twist2wheels(self, wz, vx, vy):
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

    def move(self, dx, dy, dtheta):
        vx, vy, wz = self.velocity2twist(dx, dy, dtheta)
        u = self.twist2wheels(vx, vy, wz)
        msg = Float32MultiArray(data=u)
        self.cmd_vel.publish(msg)


class OdometryReader():
    def __init__(self, topic):
        self.odom_pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.odom_pose['x'] = msg.pose.pose.position.x
        self.odom_pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.odom_pose['x'], self.odom_pose['y']))
        (_, _, self.odom_pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                 msg.pose.pose.orientation.y,
                                                                 msg.pose.pose.orientation.z,
                                                                 msg.pose.pose.orientation.w])

    def subscribe(self):
        self.odom_subscriber = rospy.Subscriber(
            self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory', self.trajectory)
        self.odom_subscriber.unregister()


def normalize(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def go_to(xg, yg, thetag_degrees, constant_vel=0.3):
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho > 0.01:
        dx = xg - odometry.odom_pose['x']
        dy = yg - odometry.odom_pose['y']
        rho = np.sqrt(dx**2 + dy**2)
        theta = odometry.odom_pose['theta']
        dtheta = normalize(thetag - theta)
        velocity.move(dx, dy, dtheta)
        rospy.sleep(0.01)


velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

waypoints = [(1, -1, -90), (2, -2, 0), (3, -2, 0), (4, -1, 90), (3.5, -0.5, 180),
             (3, 0, 90), (3, 1, 90), (2, 1, -90), (1, 0, 180), (0, 0, 180)]

for xg, yg, thetag in waypoints:
    go_to(xg, yg, thetag, constant_vel=0.3)

velocity.move(0, 0, 0)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)
