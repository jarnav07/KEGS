# ak8963.py MicroPython driver for the AK8963 magnetometer
# AK8963 is the magnetometer chip inside the MPU9250

from machine import I2C


class AK8963:
    """
    MicroPython driver for the AK8963 magnetometer
    Part of the MPU9250 9-DOF IMU
    """
    
    I2C_ADDR = 0x0C
    
    def __init__(self, i2c, address=I2C_ADDR):
        self.i2c = i2c
        self.address = address
        
    def read_id(self):
        """Read device ID"""
        return self.i2c.readfrom_mem(self.address, 0x00, 1)[0]
