#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from abluo_sdk.abluo_sdk import abluoEncoders

class EncoderROSWrapper:
    def __init__(self):
        #encoder_freq = 1
        #encoder_i2c_bus = 0
        #encoder_i2c_addr = 112
        encoder_freq = rospy.get_param("~freq")
        encoder_i2c_bus = rospy.get_param("~i2c/bus")
        encoder_i2c_addr = rospy.get_param("~i2c/addr")
        self.encoders = abluoEncoders(encoder_i2c_bus, encoder_i2c_addr)
        self.encoder_pub = rospy.Publisher("/encoder", Float32MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/encoder_freq), self.publish_encoder)

    def publish_encoder(self, event=None):
        try:
            msg = Float32MultiArray(data=self.encoders.readEncoders())
            self.encoder_pub.publish(msg)
        except Exception as e:
            print("Cannot read encoders. Check I2C connection.")

if __name__ == "__main__":
    rospy.init_node('encoder_node')
    encoder_wrapper = EncoderROSWrapper()
    rospy.spin()
