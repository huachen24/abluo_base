#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from abluo_sdk import abluoWheels

if __name__ == "__main__":
    rospy.init_node('wheel_node')
    wheels = abluoWheels(0, 0x70)
    wheel_vel_subscriber = rospy.Subscriber("/wheel_vel", Float32MultiArray, wheels.sendCommand)
    rospy.sleep(0.1)
    rospy.spin()
