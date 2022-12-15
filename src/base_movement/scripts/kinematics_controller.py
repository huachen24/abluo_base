#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


class VelocityController():
    def __init__(self, topic):
        self.vel_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        rospy.sleep(0.1)

    def velocity2twist(self, dx, dy, dtheta, thetac):
        H = np.array([[np.cos(thetac), np.sin(thetac), 0],
                    [-np.sin(thetac), np.cos(thetac), 0],
                    [0, 0, 1]])
        velocity = np.array([dx, dy, dtheta])
        velocity.shape = (3, 1)
        twist = np.dot(H, velocity)
        vx, vy, wz = twist.flatten().tolist()
        return vx, vy, wz

    def twist2wheels(self, vx, vy, wz):
        l = 0.163/2
        w = 0.215/2
        r = 0.08/2
        H = np.array([[1, -1, -w-l],
                    [1, 1, w+l],
                    [1, 1, -w-l],
                    [1, -1, w+l]]) / r
        twist = np.array([vx, vy, wz])
        twist.shape = (3, 1)
        u = np.dot(H, twist) # fl fr bl br
        return u.flatten().tolist()

    def move(self, dx, dy, dtheta, thetac):
        vx, vy, wz = self.velocity2twist(dx, dy, dtheta, thetac)
        u = self.twist2wheels(vx, vy, wz)
        msg = Float32MultiArray(data=u)
        self.vel_pub.publish(msg)

    def go_to(self, xg, yg, thetag_degrees, odom):
        rho = float("inf")
        dtheta = 5.0
        not_moving_counter = 0
        thetag = math.radians(thetag_degrees)
        while rho > 0.05 or dtheta < -0.2 or dtheta > 0.2:
            old_rho = rho
            old_dtheta = dtheta
            xc = odom.odom_pose['x']
            yc = odom.odom_pose['y']
            thetac = odom.odom_pose['theta']
            dx = xg - xc
            dy = yg - yc
            rho = np.sqrt(dx**2 + dy**2)
            dtheta = normalize(thetag - thetac)
            velocity.move(dx, dy, dtheta, thetac)
            if old_rho == rho and old_dtheta == dtheta:
                not_moving_counter += 1
            else: 
                not_moving_counter = 0
            if not_moving_counter >= 50:
                print("Robot not moving. May be stuck. ")
                break
            rospy.sleep(0.01)
        velocity.move(0, 0, 0, 0)


class OdometryReader():
    def __init__(self, topic):
        self.odom_pose = {'x': 0, 'y': 0, 'theta': 0}
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
        self.odom_subscriber.unregister()


def normalize(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def sign(number):
    if number > 0:
        return 1
    elif number < 0:
        return -1
    else:
        return 0

if __name__ == "__main__":
    try:
        rospy.init_node('kinematics_controller')
        velocity = VelocityController('/wheel_vel')
        odometry = OdometryReader('/odom')

        #testing waypoints
        waypoints = [(0, 0, 90)]
        i=0

        for xg, yg, thetag in waypoints:
            i += 1
            print("waypoint %d" % i)
            velocity.go_to(xg, yg, thetag, odometry)

        velocity.move(0, 0, 0, 0) #stop
        odometry.unregister()

        # Errors
        error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
        print('Final positioning error is %.2fm' % error)
        xs = [x[0] for x in odometry.trajectory]
        ys = [x[1] for x in odometry.trajectory]
        plt.plot(xs, ys)
        plt.show()
    except rospy.ROSInterruptException:
        velocity.move(0, 0, 0, 0) #stop
