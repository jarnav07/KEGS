"""Minimal MPU6050/MPU9250 IMU driver for MicroPython."""

from machine import I2C
from utime import sleep_ms


class MPUException(OSError):
    pass


def _u16(h, l):
    return (h << 8) | l


def _s16(h, l):
    n = (h << 8) | l
    if n & 0x8000:
        n -= 0x10000
    return n


class MPU6050:
    ADDRESSES = (0x68, 0x69)
    REG_PWR_MGMT_1 = 0x6B
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_XOUT_H = 0x3B
    REG_TEMP_OUT_H = 0x41
    REG_GYRO_XOUT_H = 0x43
    REG_WHO_AM_I = 0x75

    ACCEL_SCALE_G = (16384.0, 8192.0, 4096.0, 2048.0)
    GYRO_SCALE_DPS = (131.0, 65.5, 32.8, 16.4)

    def __init__(self, i2c, address=None, accel_range=0, gyro_range=0):
        self.i2c = i2c if hasattr(i2c, "readfrom_mem_into") else I2C(i2c)
        self.address = address if address is not None else self._detect_address()
        self._accel_range = 0
        self._gyro_range = 0

        self.wake()
        self.set_sample_rate(0)
        self.set_dlpf(0)
        self.set_accel_range(accel_range)
        self.set_gyro_range(gyro_range)

        chip_id = self.chip_id
        if chip_id not in (0x68, 0x70, 0x71):
            print("Unexpected chip ID: 0x%02x" % chip_id)

    def _detect_address(self):
        for addr in self.ADDRESSES:
            try:
                self.i2c.readfrom_mem(addr, self.REG_WHO_AM_I, 1)
                return addr
            except OSError:
                pass
        return self.ADDRESSES[0]

    def _read(self, reg, length):
        return self.i2c.readfrom_mem(self.address, reg, length)

    def _write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes((value & 0xFF,)))

    @property
    def chip_id(self):
        return self._read(self.REG_WHO_AM_I, 1)[0]

    def wake(self):
        self._write(self.REG_PWR_MGMT_1, 0x01)
        sleep_ms(50)

    def sleep(self):
        self._write(self.REG_PWR_MGMT_1, 0x40)

    def set_sample_rate(self, div):
        self._write(self.REG_SMPLRT_DIV, div)

    def set_dlpf(self, value):
        self._write(self.REG_CONFIG, value & 0x07)

    def set_accel_range(self, value):
        if value not in (0, 1, 2, 3):
            raise ValueError("accel range must be 0..3")
        self._accel_range = value
        self._write(self.REG_ACCEL_CONFIG, value << 3)

    def set_gyro_range(self, value):
        if value not in (0, 1, 2, 3):
            raise ValueError("gyro range must be 0..3")
        self._gyro_range = value
        self._write(self.REG_GYRO_CONFIG, value << 3)

    def read_accel_raw(self):
        data = self._read(self.REG_ACCEL_XOUT_H, 6)
        return (_s16(data[0], data[1]), _s16(data[2], data[3]), _s16(data[4], data[5]))

    def read_gyro_raw(self):
        data = self._read(self.REG_GYRO_XOUT_H, 6)
        return (_s16(data[0], data[1]), _s16(data[2], data[3]), _s16(data[4], data[5]))

    def read_temperature_c(self):
        data = self._read(self.REG_TEMP_OUT_H, 2)
        raw = _s16(data[0], data[1])
        return raw / 333.87 + 21.0

    def read_accel_g(self):
        raw = self.read_accel_raw()
        scale = self.ACCEL_SCALE_G[self._accel_range]
        return (raw[0] / scale, raw[1] / scale, raw[2] / scale)

    def read_accel_mps2(self):
        ax, ay, az = self.read_accel_g()
        g = 9.80665
        return (ax * g, ay * g, az * g)

    def read_gyro_dps(self):
        raw = self.read_gyro_raw()
        scale = self.GYRO_SCALE_DPS[self._gyro_range]
        return (raw[0] / scale, raw[1] / scale, raw[2] / scale)

    def read_all(self):
        return self.read_accel_mps2(), self.read_gyro_dps(), self.read_temperature_c()


class MPU9250(MPU6050):
    pass

