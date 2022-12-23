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
        H = np.array([[np.cos(thetac), -np.sin(thetac), 0],
                    [np.sin(thetac), np.cos(thetac), 0],
                    [0, 0, 1]])
        velocity = np.array([dx, dy, dtheta])
        velocity.shape = (3, 1)
        twist = np.dot(H, velocity)
        vx, vy, wz = twist.flatten().tolist()
        return vx, vy, wz

    def twist2wheels(self, vx, vy, wz):
        l = WHEEL_SEPARATION_LENGTH / 2
        w = WHEEL_SEPARATION_WIDTH / 2
        r = WHEEL_RADIUS
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
        u = [calculate_pwm(i) for i in u]
        msg = Float32MultiArray(data=u)
        self.vel_pub.publish(msg)

    def go_to(self, xg, yg, thetag_degrees, odom):
        rho = float("inf")
        dtheta = 5.0
        not_moving_counter = 0
        thetag = math.radians(thetag_degrees)
        while rho > 0.05 or dtheta < -0.15 or dtheta > 0.15:
            old_rho = rho
            old_dtheta = dtheta
            xc = odom.odom_pose['x']
            yc = odom.odom_pose['y']
            thetac = odom.odom_pose['theta']
            dx = xg - xc
            dy = yg - yc
            rho = np.sqrt(dx**2 + dy**2)
            dtheta = normalize(thetag - thetac)
            vx = sign(clamp_dist(dx)) * 0.08
            vy = sign(clamp_dist(dy)) * 0.08
            vtheta = sign(clamp_angle(dtheta)) * 0.2
            velocity.move(vx, vy, vtheta, thetac)
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
        #print(self.odom_pose['theta'])

    def subscribe(self):
        self.odom_subscriber = rospy.Subscriber(
            self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        self.odom_subscriber.unregister()


def normalize(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def clamp_dist(dist):
    if -0.05 < dist < 0.05:
        return 0
    else:
        return dist

def clamp_angle(angle):
    if -0.15 < angle < 0.15:
        return 0
    else:
        return angle

def sign(number):
    if number > 0:
        return 1
    elif number < 0:
        return -1
    else:
        return 0

def calculate_pwm(vel):
    if vel > 5:
        vel = 5
    return (vel * 2) + (15 * sign(vel))

if __name__ == "__main__":
    try:
        rospy.init_node('kinematics_controller')

        WHEEL_RADIUS = rospy.get_param("~wheel/diameter") / 2
        WHEEL_SEPARATION_WIDTH = rospy.get_param("~wheel/separation/horizontal")
        WHEEL_SEPARATION_LENGTH = rospy.get_param("~wheel/separation/vertical")

        velocity = VelocityController('/wheel_vel')
        odometry = OdometryReader('/odom')

        #testing waypoints
        waypoints = [(0.15, 0, 0)]
        i=0

        for xg, yg, thetag in waypoints:
            i += 1
            print("waypoint %d" % i)
            velocity.go_to(xg, yg, thetag, odometry)

        velocity.move(0, 0, 0, 0) #stop
        odometry.unregister()

        # Errors
        error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
        print('Final displacement is %.2fm' % error)
        print('Final angle is %.2f' % odometry.odom_pose['theta'])
        #xs = [x[0] for x in odometry.trajectory]
        #ys = [x[1] for x in odometry.trajectory]
        #plt.plot(xs, ys)
        #plt.show()
    except rospy.ROSInterruptException:
        velocity.move(0, 0, 0, 0) #stop
