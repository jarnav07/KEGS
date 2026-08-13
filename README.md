# Rocket Telemetry System

MicroPython-based rocket flight telemetry and data-logging system for a Raspberry Pi Pico-class microcontroller. The current implementation reads inertial and barometric data, estimates flight state, and records flight data to a microSD card for post-flight analysis.

> **Project status:** Active development. Hardware testing and flight validation are still required before relying on the system for a real launch.

## Features

- MPU9250 IMU measurements for acceleration and angular rate
- BMP280 barometric pressure and temperature measurements
- Roll, pitch and yaw estimation using gyro integration with accelerometer correction
- Altitude and vertical-velocity estimation using a 2-state Kalman filter
- Automatic launch detection
- Automatic apogee detection
- CSV flight-data logging to microSD
- Configurable I2C and SPI pin assignments
- Periodic serial/REPL telemetry output for live debugging

## Hardware

The code is currently configured around:

- Raspberry Pi Pico / Pico W or another MicroPython board exposing the required `machine` APIs
- GY-91-style sensor board containing an MPU9250 and BMP280
- SPI microSD card module
- microSD card for flight logging

### Pin configuration

#### I2C — IMU + barometer

| Signal | Pico GPIO |
|---|---:|
| SDA | GP4 |
| SCL | GP5 |

The I2C bus is configured for 400 kHz.

#### SPI — microSD

| Signal | Pico GPIO |
|---|---:|
| SCK | GP18 |
| MOSI | GP19 |
| MISO | GP16 |
| CS | GP17 |

The SD interface is configured for SPI bus 0 at 10 MHz.

> Check the wiring and voltage requirements of your specific modules before powering the system. Do not assume every SD module or sensor breakout is 3.3 V safe.

## Repository structure

| File | Purpose |
|---|---|
| `mainScript.py` | Main flight telemetry loop, sensor fusion, flight-state detection and CSV logger |
| `main.py` | SD-card mounting and basic file-management utilities |
| `mpu9250.py` | MPU9250 sensor driver |
| `mpu6500.py` | MPU6500 compatibility/helper module |
| `ak8963.py` | AK8963 magnetometer support used by the IMU stack |
| `bmp280.py` | BMP280 barometer/temperature driver |
| `sdcard.py` | MicroSD card driver |
| `vector3d.py` | Vector utilities used by the sensor stack |
| `imu.py` | IMU-related helper functionality |

## How the telemetry system works

1. The microcontroller starts and scans the I2C bus.
2. The MPU9250 and BMP280 are initialised.
3. The IMU is calibrated while the vehicle is stationary to estimate gyro bias and initial attitude.
4. A stationary pressure reference is collected before flight.
5. Sensor readings are updated continuously.
6. Accelerometer and gyro measurements are combined to estimate roll, pitch and yaw.
7. Vertical acceleration is estimated from the attitude-compensated accelerometer data.
8. A Kalman filter combines inertial acceleration with barometric altitude to estimate altitude and vertical velocity.
9. Launch and apogee state flags are generated from the flight data.
10. Measurements are written to a CSV file on the SD card at approximately 10 Hz while diagnostic output is printed at approximately 2 Hz.

## Data format

The telemetry CSV contains:

```text
time_ms, accel_x, accel_y, accel_z,
gyro_x, gyro_y, gyro_z,
temp_c, pressure_hpa,
altitude_m, vertical_velocity_mps,
roll_deg, pitch_deg, yaw_deg,
launched, apogee
```

The logger writes to `/sd/telemetry.csv`.

## Running on the flight computer

1. Flash a compatible MicroPython firmware to the microcontroller.
2. Copy the required Python files from this repository to the board.
3. Connect the MPU9250/BMP280 sensor bus and microSD interface according to the pin configuration above.
4. Insert a correctly formatted microSD card.
5. Keep the vehicle stationary during startup so the IMU and pressure reference can calibrate correctly.
6. Run `mainScript.py`.
7. Confirm that the sensors initialise, the pressure reference is established, and `Telemetry log opened` appears before flight.
8. After testing, remove the SD card and inspect `telemetry.csv` on a computer.

## Development and testing

This repository contains MicroPython flight software, so much of the code depends on hardware APIs such as `machine`, `utime`, and the physical sensors. Host-side execution can therefore be limited.

Before a flight, test progressively:

- Sensor detection and I2C wiring
- IMU calibration while stationary
- Pressure-reference calibration
- SD-card mounting and file creation
- CSV logging and file integrity
- Attitude response by rotating the avionics package by hand
- Altitude response using a controlled pressure change
- Launch/apogee detection against recorded or simulated data
- Full-duration powered hardware test

Flight testing should only be performed after the system has been validated on the actual hardware configuration.

## Configuration

The main hardware and timing parameters are defined near the top of `mainScript.py`, including I2C pins, SPI pins and update/logging intervals. Keep hardware-specific configuration in one place so changes are easy to review and reproduce.

## Known limitations

- Yaw is currently gyro-integrated and will drift over time unless magnetometer correction is added to the attitude solution.
- Flight-state thresholds are currently fixed and should be validated against real flight data.
- The pressure reference depends on a stable pre-launch calibration period.
- Sensor fusion parameters are currently hard-coded and will need tuning for the final airframe and sensor mounting.
- The repository currently focuses on onboard acquisition and logging; a separate ground-station/downlink implementation should be documented here if/when it is added.

## Recommended future work

- Add magnetometer-based yaw correction or a more complete AHRS solution.
- Add robust sensor fault detection and recovery.
- Improve launch/apogee detection using validated flight-data thresholds.
- Add unit tests for sensor-fusion mathematics that can run on standard Python.
- Add automated linting/formatting where compatible with the MicroPython workflow.
- Add post-flight analysis tools for plotting altitude, velocity, acceleration and attitude.
- Document the final wiring diagram and avionics mounting orientation.
- Add a hardware test checklist and release/versioning process for flight builds.

## Safety

This is experimental rocket avionics software. Sensor estimates, launch detection and apogee detection must be independently tested and validated before being used in a flight-critical system. Never treat software output alone as proof that a recovery or propulsion system is safe to operate.

## Author

**Arnav Jain** — Aerospace Engineering student and rocket avionics project developer.
