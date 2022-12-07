#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import smbus2 as smbus

class abluoArduinoApi:
    def __init__(self, i2cBus, i2cAddress, **kwargs):
        """
        Sends commands to wheel controller and get encoder data

        :param i2cBus: bus of I2C on master
        :param i2cAddress: i2c address for connected microcontroller
        """
        self._i2cBus = i2cBus
        self._i2cAddress = i2cAddress
        rospy.init_node('arduino_api')
        self.subscribe()
        self.encoder_pub = rospy.Publisher("encoder", Float32MultiArray, queue_size=10)
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            self.readEncoders()
            rate.sleep()
        rospy.spin()

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
        abluoArduino = abluoArduinoApi(1, 0x70)
    except rospy.ROSInterruptException:
        pass
