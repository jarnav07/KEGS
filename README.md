
# Kinematic Event Guidance System (KEGS)

<p align="center">
  <strong>Embedded flight-data acquisition and sensor-fusion system for model rocket avionics</strong>
</p>

<p align="center">
  <em>Raspberry Pi Pico · MicroPython · MPU9250 · BMP280 · microSD</em>
</p>

> **Project status:** Active development. Hardware testing and flight validation are still required before relying on the system for a real launch.

## Overview

This project is an onboard rocket telemetry and flight-data logging system built around a Raspberry Pi Pico-class microcontroller. It combines inertial and barometric measurements to estimate the vehicle's attitude, altitude and vertical velocity while recording flight data to a microSD card for post-flight analysis.

The software is designed as a modular avionics platform that can be progressively developed from bench testing through flight validation.

## Key capabilities

- **6-axis inertial sensing** using an MPU9250 IMU
- **Barometric altitude sensing** using a BMP280
- **Attitude estimation** for roll, pitch and yaw
- **2-state Kalman filtering** for altitude and vertical velocity estimation
- **Automatic launch detection**
- **Automatic apogee detection**
- **CSV flight-data logging** to microSD
- **Live serial/REPL diagnostic telemetry** during development and testing
- Configurable I2C and SPI hardware interfaces

## System architecture

```text
                   ┌─────────────────────┐
                   │   Raspberry Pi Pico  │
                   │     MicroPython      │
                   └──────────┬──────────┬─┘
                              │          │
                    I2C       │          │ SPI
                              │          │
                 ┌────────────▼───┐   ┌──▼─────────────┐
                 │    GY-91       │   │    microSD     │
                 │                │   │    storage     │
                 │ MPU9250 +      │   └────────────────┘
                 │ BMP280         │
                 └────────────────┘
                              │
                              ▼
                    Sensor fusion +
                    flight-state logic
                              │
                              ▼
                   altitude / velocity /
                    attitude / events
```

## Hardware

The current implementation is configured around:

- Raspberry Pi Pico / Pico W or another compatible MicroPython board
- GY-91-style sensor board containing an MPU9250 and BMP280
- SPI microSD card module
- microSD card for flight-data storage

### Pin configuration

#### I2C — IMU + barometer

| Signal | Pico GPIO |
|---|---:|
| SDA | GP4 |
| SCL | GP5 |

I2C is configured for **400 kHz**.

#### SPI — microSD

| Signal | Pico GPIO |
|---|---:|
| SCK | GP18 |
| MOSI | GP19 |
| MISO | GP16 |
| CS | GP17 |

The SD interface uses **SPI bus 0 at 10 MHz**.

> Check the wiring and voltage requirements of the specific breakout boards being used. Do not assume every SD module or sensor breakout is 3.3 V safe.

## Repository structure

| File | Purpose |
|---|---|
| `mainScript.py` | Main telemetry loop, sensor fusion, flight-state detection and CSV logger |
| `main.py` | SD-card mounting and file-management utilities |
| `mpu9250.py` | MPU9250 sensor driver |
| `mpu6500.py` | MPU6500 compatibility/helper module |
| `ak8963.py` | AK8963 magnetometer support used by the IMU stack |
| `bmp280.py` | BMP280 barometer/temperature driver |
| `sdcard.py` | MicroSD card driver |
| `vector3d.py` | Vector utilities used by the sensor stack |
| `imu.py` | IMU-related helper functionality |

## How it works

1. The microcontroller starts and scans the I2C bus.
2. The MPU9250 and BMP280 are initialised.
3. The IMU is calibrated while stationary to estimate gyro bias and initial attitude.
4. A stationary pressure reference is collected before flight.
5. Sensor measurements are sampled continuously.
6. Gyroscope and accelerometer measurements are combined to estimate roll, pitch and yaw.
7. Attitude-compensated acceleration is used to estimate vertical acceleration.
8. A **2-state Kalman filter** combines inertial acceleration with barometric altitude to estimate altitude and vertical velocity.
9. Launch and apogee detection logic monitors the estimated flight state.
10. Flight measurements are written to CSV on the microSD card at approximately **10 Hz**, while diagnostic output is printed at approximately **2 Hz**.

## Sensor fusion

The flight-state estimator maintains a state consisting of:

```text
x = [ altitude, vertical_velocity ]ᵀ
```

The prediction step uses attitude-compensated vertical acceleration, while the BMP280 provides an independent barometric altitude measurement for correction. This reduces the long-term drift that would result from integrating accelerometer data alone.

The current attitude solution uses gyro integration with accelerometer correction. Yaw is currently gyro-integrated and therefore remains susceptible to drift.

## Flight data

The telemetry logger produces a CSV file at:

```text
/sd/telemetry.csv
```

Recorded fields include:

| Field | Unit | Description |
|---|---|---|
| `time_ms` | ms | Microcontroller timestamp |
| `accel_x/y/z` | m/s² | Accelerometer measurements |
| `gyro_x/y/z` | °/s | Angular-rate measurements |
| `temp_c` | °C | BMP280 temperature |
| `pressure_hpa` | hPa | Barometric pressure |
| `altitude_m` | m | Estimated altitude |
| `vertical_velocity_mps` | m/s | Estimated vertical velocity |
| `roll_deg` | ° | Estimated roll |
| `pitch_deg` | ° | Estimated pitch |
| `yaw_deg` | ° | Estimated yaw |
| `launched` | bool | Launch-detection state |
| `apogee` | bool | Apogee-detection state |

## Setup

1. Flash compatible MicroPython firmware to the microcontroller.
2. Copy the required Python files from this repository to the board.
3. Connect the sensors and microSD interface using the pin configuration above.
4. Insert a correctly formatted microSD card.
5. Keep the avionics stationary during startup so the IMU and pressure reference can calibrate correctly.
6. Run `mainScript.py`.
7. Confirm that the sensors initialise and that `Telemetry log opened` is displayed.
8. After testing, remove the SD card and inspect `telemetry.csv` on a computer.

## Validation workflow

Because this is embedded flight software, validation should progress from low-risk tests toward full flight testing:

- [ ] Verify I2C sensor detection
- [ ] Verify IMU calibration while stationary
- [ ] Verify pressure-reference calibration
- [ ] Verify microSD mounting and file creation
- [ ] Verify CSV logging and file integrity
- [ ] Check attitude response by rotating the avionics package by hand
- [ ] Check altitude response using a controlled pressure change
- [ ] Validate launch detection against recorded/simulated data
- [ ] Validate apogee detection against recorded/simulated data
- [ ] Perform a full-duration powered hardware test
- [ ] Conduct flight testing only after the above checks pass

## Development notes

The flight software targets MicroPython and therefore depends on hardware APIs such as `machine` and `utime`. Full host-side execution is not expected for all modules.

Hardware-specific configuration is intentionally kept near the top of `mainScript.py`, making pin assignments and timing parameters easier to review and reproduce.

## Current limitations

- Yaw is gyro-integrated and will drift without magnetometer/AHRS correction.
- Launch and apogee thresholds are currently fixed and require validation against real flight data.
- Pressure-reference calibration requires the vehicle to remain stationary before launch.
- Sensor-fusion parameters are currently hard-coded and require tuning for the final airframe and sensor mounting.
- The repository currently focuses on onboard acquisition and logging rather than a radio downlink/ground-station system.

## Roadmap

- [ ] Improve yaw estimation with magnetometer correction / AHRS
- [ ] Add sensor fault detection and recovery
- [ ] Tune launch and apogee detection using flight data
- [ ] Add host-side unit tests for sensor-fusion mathematics
- [ ] Add post-flight plotting and analysis tools
- [ ] Document final avionics wiring and mechanical orientation
- [ ] Add a formal hardware test checklist
- [ ] Introduce tagged flight-software releases
- [ ] Add radio telemetry and a dedicated ground-station interface

## Safety

This is experimental rocket avionics software. Sensor estimates, launch detection and apogee detection must be independently tested and validated before being used in any flight-critical system. Software output must never be treated as proof that a propulsion or recovery system is safe to operate.

## Author

**Arnav Jain** — Aerospace Engineering student and rocket avionics project developer.

---

*Built as an experimental embedded-systems and aerospace engineering project, with an emphasis on sensor fusion, flight-data acquisition and practical avionics development.*
