#!/usr/bin/env python3
import rospy
import enum
from tokenize import String
import smbus2 as smbus
import time

rospy.init_node('wheel_api')

class abluoWheels:
    class WHEEL_INDEX():
        FRONT_LEFT = 0
        FRONT_RIGHT = 1
        BACK_LEFT = 2
        BACK_RIGHT = 3
    
    class WHEEL_DIRECTION():
        FORWARD = 0
        BACKWARD = 1

    def __init__(self, i2cBus, i2cAddress, **kwargs):
        """
        Sends commands to wheel controller

        :param i2cBus: bus of I2C on master
        :param i2cAddress: i2c address for connected microcontroller
        """
        self._i2cBus = i2cBus
        self._i2cAddress = i2cAddress
        self.subscribe()

    def sendCommand(self, msg):
        payload = "{},{},{},{}\n".format(msg[0], msg[1], msg[2], msg[3])
        payload_byte = []
        for c in payload:
            payload_byte.append(ord(c))
        try:
            wire = smbus.SMBus(self._i2cBus)
            wire.write_i2c_block_data(self._i2cAddress, 0x0, payload_byte)
        except IOError:
            # Emergency stop handling
            print("Wheel microcontroller not detected")

    def subscribe(self):
        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Float32MultiArray, self.sendCommand)
        rospy.sleep(0.1)

    def unregister(self):
        stop = [0, 0, 0, 0]
        self.sendCommand(stop)
        self.cmd_vel_subscriber.unregister()

if __name__ == "__main__":
    abluoWheels = abluoWheelsApi(1, 0x70)