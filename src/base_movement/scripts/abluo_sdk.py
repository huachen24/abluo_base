#!/usr/bin/env python
import smbus2 as smbus

class abluoWheels:
    def __init__(self, i2cBus, i2cAddress, **kwargs):
        """
        Sends commands to wheel controller

        :param i2cBus: bus of I2C on master
        :param i2cAddress: i2c address for connected microcontroller
        """
        self._i2cBus = i2cBus
        self._i2cAddress = i2cAddress

    def sendCommand(self, msg):
        vel = msg.data
        vel = [round(i, 2) for i in vel]
        payload = "{:07.2f},{:07.2f},{:07.2f},{:07.2f}\n".format(vel[0], vel[1], vel[2], vel[3])
        payload_byte = []
        for c in payload:
            payload_byte.append(ord(c))
        try:
            wire = smbus.SMBus(self._i2cBus)
            wire.write_i2c_block_data(self._i2cAddress, 0x0, payload_byte)
        except IOError:
            # Emergency stop handling
            print("Wheel microcontroller not detected")

class abluoEncoders:
    def __init__(self, i2cBus, i2cAddress, **kwargs):
        """
        Get encoder data

        :param i2cBus: bus of I2C on master
        :param i2cAddress: i2c address for connected microcontroller
        """
        self._i2cBus = i2cBus
        self._i2cAddress = i2cAddress

    def readEncoders(self):
        with smbus.SMBus(self._i2cBus) as wire:
            read = smbus.i2c_msg.read(self._i2cAddress, 31)
            wire.i2c_rdwr(read)
        spdbytes = list(read)
        spdstring = bytes(spdbytes).decode('ascii')
        velocities = spdstring[1:].split(',')
        velocities = [float(vel) for vel in velocities]
        return velocities
