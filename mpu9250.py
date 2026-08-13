"""MPU9250 wrapper built on the 6-axis MPU6050 core."""

from imu import MPU6050


class MPU9250(MPU6050):
    """Use the accel/gyro core of MPU9250-compatible devices.

    Many clone boards expose a broken or missing magnetometer, so this driver
    intentionally focuses on the 6-axis IMU registers only.
    """

    pass

