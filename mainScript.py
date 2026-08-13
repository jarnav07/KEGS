"""Rocket telemetry logger for a GY-91 style board."""

from machine import I2C, Pin
from math import atan2, cos, degrees, radians, sin, sqrt
import utime

from bmp280 import BMP280
from mpu9250 import MPU9250


# Configurable pin assignments
# I2C (sensors)
I2C_ID = 0
SDA_PIN = 4
SCL_PIN = 5

# SD card SPI pins (match main.py)
SPI_ID = 0
SCK_PIN = 18
MOSI_PIN = 19
MISO_PIN = 16
CS_PIN = 17


G = 9.80665
CONSOLE_INTERVAL_MS = 500
LOG_INTERVAL_MS = 100
UPDATE_INTERVAL_MS = 20


class TelemetryLogger:
    def __init__(self, telemetry, filename="/sd/telemetry.csv"):
        self.telemetry = telemetry
        self.filename = filename
        self.handle = None

    def open(self):
        try:
            self.handle = open(self.filename, "w")
            self.handle.write(self.telemetry.csv_header() + "\n")
            print("Telemetry log opened: %s" % self.filename)
            return True
        except OSError as e:
            print("SD logging disabled: %s" % e)
            # Attempt to mount the SD card and retry opening (useful on MicroPython hardware)
            try:
                import main as sdmod
            except Exception:
                sdmod = None
            if sdmod is not None:
                try:
                    sdmod.mount_sd_card()
                    # retry opening after mount
                    try:
                        self.handle = open(self.filename, "w")
                        self.handle.write(self.telemetry.csv_header() + "\n")
                        print("Telemetry log opened after mounting SD: %s" % self.filename)
                        return True
                    except Exception as open_exc:
                        print("Failed to open telemetry file after mounting SD:", open_exc)
                except Exception as mount_exc:
                    print("Failed to mount SD card:", mount_exc)
            self.handle = None
            return False

    def log(self):
        if self.handle is None:
            return
        self.handle.write(self.telemetry.csv_row() + "\n")

    def flush(self):
        if self.handle is not None:
            self.handle.flush()

    def close(self):
        if self.handle is not None:
            self.handle.close()
            self.handle = None


class RocketTelemetry:
    def __init__(self, i2c_id=I2C_ID, sda_pin=SDA_PIN, scl_pin=SCL_PIN):
        self.i2c = I2C(i2c_id, freq=400000, sda=Pin(sda_pin), scl=Pin(scl_pin))

        devices = self.i2c.scan()
        print("I2C scan:", [hex(x) for x in devices])

        self.imu = MPU9250(self.i2c)
        print("IMU initialized at 0x%02x" % self.imu.address)

        self.bmp = BMP280(self.i2c)
        print("BMP280 initialized at 0x%02x" % self.bmp.address)

        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.temperature = 0.0
        self.pressure = 0.0
        self.altitude = 0.0
        self.altitude_prev = 0.0
        self.altitude_max = 0.0
        self.vertical_velocity = 0.0
        self.velocity_max = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.launch_detected = False
        self.apogee_detected = False

        self.gyro_bias = (0.0, 0.0, 0.0)
        self.pressure_ref = None
        self.last_update_ms = utime.ticks_ms()
        self.accel_filt = (0.0, 0.0, 0.0)
        self.launch_counter = 0
        self.altitude_ready = False
        self.vertical_accel = 0.0

        self.kf_x = [[0.0], [0.0]]  # altitude, vertical velocity
        self.kf_P = [[20.0, 0.0], [0.0, 20.0]]
        self.kf_accel_var = 4.0
        self.kf_baro_var = 9.0

        self._roll_ready = False
        self._pitch_ready = False

        self.calibrate_imu()

    def calibrate_imu(self, samples=100, delay_ms=10):
        gx = gy = gz = 0.0
        ax = ay = az = 0.0
        for _ in range(samples):
            a = self.imu.read_accel_mps2()
            g = self.imu.read_gyro_dps()
            ax += a[0]
            ay += a[1]
            az += a[2]
            gx += g[0]
            gy += g[1]
            gz += g[2]
            utime.sleep_ms(delay_ms)

        self.gyro_bias = (gx / samples, gy / samples, gz / samples)
        self.roll = degrees(atan2(ay / samples, az / samples))
        self.pitch = degrees(atan2(-(ax / samples), sqrt((ay / samples) ** 2 + (az / samples) ** 2)))
        self._roll_ready = True
        self._pitch_ready = True
        print("IMU calibrated")

    def calibrate_pressure(self, samples=20, delay_ms=50):
        readings = []
        for _ in range(samples):
            pressure = self.bmp.read_pressure()
            if 300.0 <= pressure <= 1200.0:
                readings.append(pressure)
            utime.sleep_ms(delay_ms)

        if not readings:
            print("BMP280 pressure invalid; altitude will use velocity fallback")
            self.pressure_ref = None
            return False

        self.pressure_ref = sum(readings) / len(readings)
        self.altitude = 0.0
        self.altitude_max = 0.0
        print("Pressure reference: %.2f hPa" % self.pressure_ref)
        return True

    def _read_sensors(self):
        accel = self.imu.read_accel_mps2()
        self.accel = accel
        alpha = 0.2
        self.accel_filt = (
            alpha * accel[0] + (1.0 - alpha) * self.accel_filt[0],
            alpha * accel[1] + (1.0 - alpha) * self.accel_filt[1],
            alpha * accel[2] + (1.0 - alpha) * self.accel_filt[2],
        )
        self.gyro = self.imu.read_gyro_dps()
        self.temperature = self.bmp.read_temperature()
        self.pressure = self.bmp.read_pressure()

    def _estimate_vertical_accel(self):
        ax, ay, az = self.accel_filt
        roll = radians(self.roll)
        pitch = radians(self.pitch)
        world_z = (
            -sin(pitch) * ax
            + sin(roll) * cos(pitch) * ay
            + cos(roll) * cos(pitch) * az
        )
        return world_z - G

    def _kf_predict(self, accel_z, dt):
        x0 = self.kf_x[0][0]
        x1 = self.kf_x[1][0]

        x0_new = x0 + x1 * dt + 0.5 * accel_z * dt * dt
        x1_new = x1 + accel_z * dt
        self.kf_x[0][0] = x0_new
        self.kf_x[1][0] = x1_new

        p00 = self.kf_P[0][0]
        p01 = self.kf_P[0][1]
        p10 = self.kf_P[1][0]
        p11 = self.kf_P[1][1]

        q00 = 0.25 * dt ** 4 * self.kf_accel_var
        q01 = 0.5 * dt ** 3 * self.kf_accel_var
        q11 = dt ** 2 * self.kf_accel_var

        p00_new = p00 + dt * (p10 + p01) + dt * dt * p11 + q00
        p01_new = p01 + dt * p11 + q01
        p10_new = p10 + dt * p11 + q01
        p11_new = p11 + q11

        self.kf_P[0][0] = p00_new
        self.kf_P[0][1] = p01_new
        self.kf_P[1][0] = p10_new
        self.kf_P[1][1] = p11_new

    def _kf_update_altitude(self, measured_altitude):
        p00 = self.kf_P[0][0]
        p01 = self.kf_P[0][1]
        p10 = self.kf_P[1][0]
        p11 = self.kf_P[1][1]

        s = p00 + self.kf_baro_var
        if s == 0.0:
            return

        k0 = p00 / s
        k1 = p10 / s
        y = measured_altitude - self.kf_x[0][0]

        self.kf_x[0][0] += k0 * y
        self.kf_x[1][0] += k1 * y

        self.kf_P[0][0] = (1.0 - k0) * p00
        self.kf_P[0][1] = (1.0 - k0) * p01
        self.kf_P[1][0] = p10 - k1 * p00
        self.kf_P[1][1] = p11 - k1 * p01

    def _update_orientation(self, dt):
        ax, ay, az = self.accel_filt
        gx, gy, gz = self.gyro
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]
        self.gyro = (gx, gy, gz)

        roll_acc = degrees(atan2(ay, az))
        pitch_acc = degrees(atan2(-ax, sqrt(ay * ay + az * az)))

        alpha = 0.98
        self.roll = alpha * (self.roll + gx * dt) + (1.0 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + gy * dt) + (1.0 - alpha) * pitch_acc
        self.yaw += gz * dt

    def _update_altitude(self, dt ):
        self.vertical_accel = self._estimate_vertical_accel()
        if abs(self.vertical_accel) < 0.25:
            self.vertical_accel = 0.0

        self._kf_predict(self.vertical_accel, dt)

        if self.pressure_ref is not None and 300.0 <= self.pressure <= 1200.0:
            baro_altitude = 44330.0 * (1.0 - (self.pressure / self.pressure_ref) ** 0.190294957)
            if not self.altitude_ready:
                self.kf_x[0][0] = baro_altitude
                self.kf_x[1][0] = 0.0
                self.altitude_ready = True
            else:
                self._kf_update_altitude(baro_altitude)

        self.altitude = self.kf_x[0][0]
        self.vertical_velocity = self.kf_x[1][0]

        if self.altitude > self.altitude_max:
            self.altitude_max = self.altitude

    def _update_velocity(self, dt):
        """Backward-compatible wrapper for the Kalman-based motion update."""
        self._update_altitude(dt)

    def _update_state_flags(self):
        ax, ay, az = self.accel_filt
        magnitude = sqrt(ax * ax + ay * ay + az * az)
        if not self.launch_detected:
            if abs(magnitude - G) > 4.0:
                self.launch_counter += 1
            else:
                self.launch_counter = 0
            if self.launch_counter >= 5:
                self.launch_detected = True
                print("LAUNCH DETECTED")

        if self.launch_detected and not self.apogee_detected and self.vertical_velocity < -0.2:
            self.apogee_detected = True
            print("APOGEE DETECTED: %.1f m" % self.altitude_max)

    def update(self):
        now = utime.ticks_ms()
        dt = (now - self.last_update_ms) / 1000.0
        if dt <= 0.0:
            return
        self.last_update_ms = now

        self._read_sensors()
        self._update_orientation(dt)
        self._update_altitude(dt)
        self._update_state_flags()

    def csv_header(self):
        return "time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,pressure_hpa,altitude_m,vertical_velocity_mps,roll_deg,pitch_deg,yaw_deg,launched,apogee"

    def csv_row(self):
        now = utime.ticks_ms()
        return ",".join([
            str(now),
            "%.4f" % self.accel[0],
            "%.4f" % self.accel[1],
            "%.4f" % self.accel[2],
            "%.4f" % self.gyro[0],
            "%.4f" % self.gyro[1],
            "%.4f" % self.gyro[2],
            "%.2f" % self.temperature,
            "%.2f" % self.pressure,
            "%.2f" % self.altitude,
            "%.4f" % self.vertical_velocity,
            "%.2f" % self.roll,
            "%.2f" % self.pitch,
            "%.2f" % self.yaw,
            str(self.launch_detected),
            str(self.apogee_detected),
        ])


def main():
    print("Initializing rocket telemetry...")
    telemetry = RocketTelemetry()
    logger = TelemetryLogger(telemetry)

    print("Calibrating pressure sensor...")
    telemetry.calibrate_pressure()
    logger.open()

    last_console = utime.ticks_ms()
    last_log = utime.ticks_ms()
    print("Streaming telemetry...")

    try:
        while True:
            telemetry.update()
            now = utime.ticks_ms()

            if now - last_log >= LOG_INTERVAL_MS:
                logger.log()
                last_log = now

            if now - last_console >= CONSOLE_INTERVAL_MS:
                print(
                    "[%dms] alt=%.1fm vz=%.2fm/s roll=%.1f pitch=%.1f yaw=%.1f ax=%.2f ay=%.2f az=%.2f p=%.1f"
                    % (
                        now,
                        telemetry.altitude,
                        telemetry.vertical_velocity,
                        telemetry.roll,
                        telemetry.pitch,
                        telemetry.yaw,
                        telemetry.accel[0],
                        telemetry.accel[1],
                        telemetry.accel[2],
                        telemetry.pressure,
                    )
                )
                last_console = now

            utime.sleep_ms(UPDATE_INTERVAL_MS)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        logger.close()


if __name__ == "__main__":
    main()
