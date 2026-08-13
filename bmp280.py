"""Minimal BMP280 driver for MicroPython."""

from machine import I2C
from utime import sleep_ms


class BMP280:
    ADDRESSES = (0x76, 0x77)
    REG_CALIB = 0x88
    REG_ID = 0xD0
    REG_RESET = 0xE0
    REG_STATUS = 0xF3
    REG_CTRL_MEAS = 0xF4
    REG_CONFIG = 0xF5
    REG_PRESS_MSB = 0xF7

    CHIP_ID = 0x58

    def __init__(self, i2c, address=None):
        self.i2c = i2c if hasattr(i2c, "readfrom_mem_into") else I2C(i2c)
        self.address = address if address is not None else self._detect_address()
        self._t_fine = 0.0
        self._read_calibration()
        self._init_sensor()
        chip_id = self.read_id()
        if chip_id != self.CHIP_ID:
            print("Unexpected BMP280 chip ID: 0x%02x" % chip_id)

    def _detect_address(self):
        for addr in self.ADDRESSES:
            try:
                self.i2c.readfrom_mem(addr, self.REG_ID, 1)
                return addr
            except OSError:
                pass
        return self.ADDRESSES[0]

    def _read_u16_le(self, reg):
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return data[0] | (data[1] << 8)

    def _read_s16_le(self, reg):
        value = self._read_u16_le(reg)
        if value & 0x8000:
            value -= 0x10000
        return value

    def _read_calibration(self):
        calib = self.i2c.readfrom_mem(self.address, self.REG_CALIB, 24)
        self.dig_T1 = calib[0] | (calib[1] << 8)
        self.dig_T2 = _s16(calib[2], calib[3])
        self.dig_T3 = _s16(calib[4], calib[5])
        self.dig_P1 = calib[6] | (calib[7] << 8)
        self.dig_P2 = _s16(calib[8], calib[9])
        self.dig_P3 = _s16(calib[10], calib[11])
        self.dig_P4 = _s16(calib[12], calib[13])
        self.dig_P5 = _s16(calib[14], calib[15])
        self.dig_P6 = _s16(calib[16], calib[17])
        self.dig_P7 = _s16(calib[18], calib[19])
        self.dig_P8 = _s16(calib[20], calib[21])
        self.dig_P9 = _s16(calib[22], calib[23])

    def _init_sensor(self):
        self.i2c.writeto_mem(self.address, self.REG_CTRL_MEAS, bytes((0x27,)))
        self.i2c.writeto_mem(self.address, self.REG_CONFIG, bytes((0x00,)))
        sleep_ms(50)

    def read_id(self):
        return self.i2c.readfrom_mem(self.address, self.REG_ID, 1)[0]

    def _read_raw(self):
        data = self.i2c.readfrom_mem(self.address, self.REG_PRESS_MSB, 6)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return adc_p, adc_t

    def _compensate_temp(self, adc_t):
        var1 = (adc_t / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_t / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        self._t_fine = var1 + var2
        return self._t_fine / 5120.0

    def _compensate_pressure(self, adc_p):
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            return 0.0
        p = 1048576.0 - adc_p
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        p = p + (var1 + var2 + self.dig_P7) / 16.0
        return p

    def read_temperature(self):
        _, adc_t = self._read_raw()
        return self._compensate_temp(adc_t)

    def read_pressure(self):
        adc_p, adc_t = self._read_raw()
        self._compensate_temp(adc_t)
        return self._compensate_pressure(adc_p) / 100.0

    def read_altitude(self, reference_pressure_hpa=1013.25):
        pressure = self.read_pressure()
        if pressure <= 0 or reference_pressure_hpa <= 0:
            return 0.0
        return 44330.0 * (1.0 - (pressure / reference_pressure_hpa) ** 0.190294957)


def _s16(msb, lsb):
    value = (lsb << 8) | msb
    if value & 0x8000:
        value -= 0x10000
    return value

