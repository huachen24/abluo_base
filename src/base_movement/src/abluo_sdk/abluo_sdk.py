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

    def sendCommand(self, vel):
        vel = [int(i) for i in vel]
        payload = "{:03d},{:03d},{:03d},{:03d}\n".format(vel[0], vel[1], vel[2], vel[3])
        payload_byte = []
        for c in payload:
            payload_byte.append(ord(c))
        wire = smbus.SMBus(self._i2cBus)
        wire.write_i2c_block_data(self._i2cAddress, 0x00, payload_byte)

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
            read = wire.read_i2c_block_data(self._i2cAddress, 0x00, 31)
        spdbytes = list(read)
        spdstring = bytearray(spdbytes).decode("ascii")
        velocities = spdstring[1:].split(',')
        velocities = [int(vel) for vel in velocities]
        return velocities
