#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped

from math import cos, sin, pi

ticks_front_left = 0
ticks_front_right = 0
ticks_rear_left = 0
ticks_rear_right = 0

position_front_left = 0
position_front_right = 0
position_rear_left = 0
position_rear_right = 0

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

time = None


def publish_joint_state(encoder, new_time):
    global ticks_front_left
    global ticks_front_right
    global ticks_rear_left
    global ticks_rear_right

    global position_front_left
    global position_front_right
    global position_rear_left
    global position_rear_right

    global time

    wheels_state = JointState()
    wheels_state.header.stamp = rospy.Time.now()

    wheels_state.name = [
        'base_link_to_wheel_front_left',
        'base_link_to_wheel_front_right',
        'base_link_to_wheel_rear_left',
        'base_link_to_wheel_rear_right'
    ]

    ang_vel_front_left = encoder.data[0]
    ang_vel_front_right = encoder.data[1]
    ang_vel_rear_left = encoder.data[2]
    ang_vel_rear_right = encoder.data[3]

    v_front_left = compute_horizontal_velocity(ang_vel_front_left)
    v_front_right = compute_horizontal_velocity(ang_vel_front_right)
    v_rear_left = compute_horizontal_velocity(ang_vel_rear_left)
    v_rear_right = compute_horizontal_velocity(ang_vel_rear_right)

    wheels_state.velocity = [
        v_front_left,
        v_front_right,
        v_rear_left,
        v_rear_right
    ]

    position_front_left += compute_distance(v_front_left, new_time)
    position_front_right += compute_distance(v_front_right, new_time)
    position_rear_left += compute_distance(v_rear_left, new_time)
    position_rear_right += compute_distance(v_rear_right, new_time)

    wheels_state.position = [
        position_front_left,
        position_front_right,
        position_rear_left,
        position_rear_right
    ]

    joint_states_pub.publish(wheels_state)

    return v_front_left, v_front_right, v_rear_left, v_rear_right


def broadcast_transform(x, y, th, time):
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    rotation = tf.transformations.quaternion_from_euler(0, 0, th)

    odom_broadcaster.sendTransform(
        (x, y, 0),
        rotation,
        time,
        "base_link",
        "odom"
    )


def publish_odom(x, y, th, vx, vy, vth, time):
    odom_msg = Odometry()

    odom_msg.header.stamp = time
    odom_msg.header.frame_id = "odom"

    # set the position
    odom_msg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, th)))

    # set the velocity
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom_msg)


def compute_distance(velocity, new_time):
    return velocity * (new_time - time).to_sec()

def compute_horizontal_velocity(ang_vel):
    return ang_vel * WHEEL_RADIUS

def update(encoder):
    global vx, vy, vth
    global x, y, th
    global time

    new_time = rospy.Time.now()

    v_front_left, v_front_right, v_rear_left, v_rear_right = publish_joint_state(encoder, new_time)

    # From wheel velocities, compute base velocity by using the inverse kinematics
    vx = (v_front_left + v_front_right + v_rear_left + v_rear_right) * WHEEL_RADIUS / 4
    vy = (-v_front_left + v_front_right + v_rear_left - v_rear_right) * WHEEL_RADIUS / 4
    vth = (-v_front_left + v_front_right - v_rear_left + v_rear_right) * WHEEL_RADIUS / (2 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH))

    dt = (new_time - time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    broadcast_transform(x, y, th, new_time)
    publish_odom(x, y, th, vx, vy, vth, new_time)

    time = new_time

if __name__ == '__main__':
    try:
        rospy.init_node('jointstate_node')

        WHEEL_RADIUS = rospy.get_param("~wheel/diameter") / 2
        WHEEL_SEPARATION_WIDTH = rospy.get_param("~wheel/separation/horizontal")
        WHEEL_SEPARATION_LENGTH = rospy.get_param("~wheel/separation/vertical")

        encoder_sub = rospy.Subscriber('/encoder', Float32MultiArray, update)

        joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=50)
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)

        odom_broadcaster = tf.TransformBroadcaster()

        time = rospy.Time.now()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
