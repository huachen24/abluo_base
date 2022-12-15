#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from abluo_sdk.abluo_sdk import abluoEncoders

class FakeMovement:
    def __init__(self):
        encoder_freq = rospy.get_param("~freq")
        self.encoder_data = [0, 0, 0, 0]
        rospy.Subscriber("/wheel_vel", Float32MultiArray, self.convert_vel2enc)
        self.encoder_pub = rospy.Publisher("/encoder", Int32MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/encoder_freq), self.publish_encoder)

    def publish_encoder(self, event=None):
        try:
            msg = Int32MultiArray(data=self.encoder_data)
            self.encoder_pub.publish(msg)
        except Exception as e:
            print(e)
            print(self.encoder_data)
            # print("Cannot read encoders. Check I2C connection.")

    def convert_vel2enc(self, msg):
        vel = msg.data
        for i in range(len(vel)):
            self.encoder_data[i] += int(vel[i] * 0.01 / 0.05 / 3.14159265 * 500)

if __name__ == "__main__":
    rospy.init_node('fake_node')
    fake_node = FakeMovement()
    rospy.spin()
