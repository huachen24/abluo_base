#!/usr/bin/env python3
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class VelocityController():
    def __init__(self, topic):
        self.vel_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)
        rospy.sleep(0.1)

    def velocity2twist(self, dx, dy, dtheta, thetac):
        H = np.array([[np.cos(thetac), -np.sin(thetac), 0],
                    [np.sin(thetac), np.cos(thetac), 0],
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

    def move(self, dx, dy, dtheta, thetac):
        vx, vy, wz = self.velocity2twist(dx, dy, dtheta, thetac)
        u = self.twist2wheels(vx, vy, wz)
        msg = Float32MultiArray(data=u)
        self.vel_pub.publish(msg)

    def go_to(self, xg, yg, thetag_degrees, odom, constant_vel=0.3):
        rho = float("inf")
        not_moving_counter = 0
        thetag = math.radians(thetag_degrees)
        while rho > 0.01:
            old_rho = rho
            xc = odom.odom_pose['x']
            yc = odom.odom_pose['y']
            thetac = odom.odom_pose['theta']
            dx = xg - xc
            dy = yg - yc
            rho = np.sqrt(dx**2 + dy**2)
            dtheta = normalize(thetag - thetac)
            velocity.move(dx, dy, dtheta, thetac)
            if old_rho == rho:
                not_moving_counter += 1
            if not_moving_counter >= 3:
                print("Robot not moving. May be stuck. ")
                break
            rospy.sleep(0.01)


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


def normalize(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


if __name__ == "__main__":
    try:
        rospy.init_node('kinematics_controller')
        velocity = VelocityController('/wheel_vel')
        odometry = OdometryReader('/odom')

        #testing waypoints
        waypoints = [(1, -1, 0), (0, 0, 180)]

        for xg, yg, thetag in waypoints:
            velocity.go_to(xg, yg, thetag, odometry, constant_vel=0.3)

        velocity.move(0, 0, 0, 0) #stop

        # Errors
        error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
        print('Final positioning error is %.2fm' % error)
    except rospy.ROSInterruptException:
        pass