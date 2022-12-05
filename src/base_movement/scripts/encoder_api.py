#!/usr/bin/env python3
import rospy
import enum
from std_msgs.msg import Float32MultiArray
from tokenize import String
import smbus2 as smbus
import time

class abluoEncodersApi:
    def __init__(self, i2cBus, i2cAddress, **kwargs):
        """
        Gets encoder data from the wheels

        :param i2cBus: bus of I2C on master
        :param i2cAddress: i2c address for connected microcontroller
        """
        self._i2cBus = i2cBus
        self._i2cAddress = i2cAddress
        rospy.init_node('encoder_api')
        self.encoder_pub = rospy.Publisher("encoder", Float32MultiArray, queue_size=10)
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            self.readEncoders()
            rate.sleep()

    def readEncoders(self):
        wire = smbus.SMBus(self._i2cBus)
        read = smbus.i2c_msg.read(self._i2cAddress, 31)
        wire.i2c_rdwr(read)
        spdbytes = list(read)
        spdstring = bytes(spdbytes).decode('ascii')
        velocities = spdstring[1:].split(',')
        msg = Float32MultiArray(data=velocities)
        self.encoder_pub.publish(msg)


if __name__ == "__main__":
    try:
        abluoEncoders = abluoEncodersApi(1, 0x65)
    except rospy.ROSInterruptException:
        pass
