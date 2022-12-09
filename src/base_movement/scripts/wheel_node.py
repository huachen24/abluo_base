#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from abluo_sdk.abluo_sdk import abluoWheels

class WheelROSWrapper:
    def __init__(self):
        wheel_i2c_bus = rospy.get_param("~i2c/bus")
        wheel_i2c_addr = rospy.get_param("~i2c/addr")
        self.wheels = abluoWheels(wheel_i2c_bus, wheel_i2c_addr)
        rospy.Subscriber("/wheel_vel", Float32MultiArray, self.callback_send_command)

    def callback_send_command(self, msg):
        self.wheels.sendCommand(msg.data)

    def stop(self):
        self.wheels.sendCommand([0.0, 0.0, 0.0, 0.0])

if __name__ == "__main__":
    rospy.init_node('wheel_node')
    wheel_wrapper = WheelROSWrapper()
    rospy.on_shutdown(wheel_wrapper.stop)
    rospy.spin()
