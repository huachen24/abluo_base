#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from abluo_sdk import abluoEncoders

if __name__ == "__main__":
    rospy.init_node('encoder_node')
    encoder = abluoEncoders(0, 0x70)
    encoder_pub = rospy.Publisher("/encoder", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        velocities = encoder.readEncoders()
        msg = Float32MultiArray(data=velocities)
        encoder_pub.publish(msg)
        rate.sleep()
