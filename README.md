# Helicopter Flight Controller — ESP32

An experimental **embedded flight-controller project** built around an ESP32, currently focused on developing the sensor-processing and attitude-estimation layer for a helicopter flight-control system.

The project combines an IMU, magnetometer, sensor calibration, quaternion-based attitude estimation, and Wi-Fi telemetry into a single ESP32 firmware application.

> **Project Status:** Experimental / On Hold
> Development is currently paused due to financial constraints on the overall helicopter project.

---

## Overview

The goal of this project is to develop the embedded control and sensing infrastructure required for a helicopter flight controller.

The current prototype focuses primarily on **attitude sensing and estimation** rather than complete autonomous flight control.

The ESP32 collects data from:

* **MPU6050** — accelerometer and gyroscope
* **QMC5883 / compatible magnetometer** — magnetic-field measurements

The sensor data is processed to estimate the helicopter's:

* Roll
* Pitch
* Yaw

The estimated orientation is then made available through a lightweight HTTP server running directly on the ESP32.

---

## Current Architecture

```text
                 ┌──────────────────┐
                 │      ESP32       │
                 │   Flight MCU     │
                 └────────┬─────────┘
                          │
             ┌────────────┴────────────┐
             │                         │
             ▼                         ▼
      ┌─────────────┐          ┌──────────────┐
      │   MPU6050   │          │   QMC5883    │
      │             │          │ Magnetometer │
      │ Acc + Gyro  │          │              │
      └──────┬──────┘          └──────┬───────┘
             │                        │
             └───────────┬────────────┘
                         ▼
               ┌──────────────────┐
               │ Sensor Processing│
               │ & Calibration    │
               └────────┬─────────┘
                        ▼
               ┌──────────────────┐
               │ Madgwick Filter  │
               │ Quaternion State │
               │    Estimation    │
               └────────┬─────────┘
                        ▼
               ┌──────────────────┐
               │ Euler Conversion │
               └────────┬─────────┘
                        │
                ┌───────┴────────┐
                ▼                ▼
          Serial Output      HTTP Server
                             ESP32 Wi-Fi
```

---

## Features

### IMU Processing

The firmware communicates with the MPU6050 and obtains:

* 3-axis accelerometer measurements
* 3-axis gyroscope measurements

The current implementation configures the sensor directly through I²C registers and converts the raw measurements into usable acceleration and angular-rate values.

The accelerometer data is additionally smoothed using a simple low-pass filter before being supplied to the attitude estimator.

---

### Magnetometer Calibration

The magnetometer is calibrated during startup.

The firmware collects measurements for approximately **15 seconds** while the sensor is rotated through different orientations.

During calibration it records:

```text
X minimum / maximum
Y minimum / maximum
Z minimum / maximum
```

These values are subsequently used for basic hard-iron offset correction.

A minimum sample count is also enforced before calibration is accepted.

---

### Madgwick Attitude Estimation

The project contains a custom implementation of the **Madgwick attitude-estimation algorithm**.

Rather than directly integrating Euler angles, the filter maintains orientation as a quaternion:

```text
q = [q0, q1, q2, q3]
```

The quaternion is updated using:

* Gyroscope angular velocity
* Accelerometer measurements
* Magnetometer measurements

The resulting quaternion is normalized and converted into Euler angles for easier interpretation:

```text
Roll
Pitch
Yaw
```

This approach avoids many of the numerical problems associated with directly integrating Euler angles.

---

## Wi-Fi Telemetry

The ESP32 also runs a lightweight HTTP server.

After connecting to Wi-Fi, the firmware exposes the current orientation through the root endpoint:

```text
GET /
```

The response currently follows the format:

```text
PITCH:<value>,ROLL:<value>
```

For example:

```text
PITCH:-2.34,ROLL:5.81
```

This makes it possible for another device on the same network to retrieve the current attitude information from the flight-controller prototype.

CORS headers are also enabled for the endpoint, allowing browser-based clients to consume the data.

---

## Hardware

The current firmware is designed around:

| Component                   | Purpose                                              |
| --------------------------- | ---------------------------------------------------- |
| ESP32 DevKit V1             | Main microcontroller                                 |
| MPU6050                     | Accelerometer + gyroscope                            |
| QMC5883 / compatible sensor | Magnetometer                                         |
| Servo motors                | Intended future control-surface / actuator interface |

The PlatformIO configuration currently targets:

```ini
board = esp32doit-devkit-v1
framework = arduino
```

and uses the Espressif32 PlatformIO platform.

---

## Software Stack

### Platform

* PlatformIO
* ESP32
* Arduino framework
* C++

### Sensors / Libraries

The current `platformio.ini` declares:

* `electroniccats/MPU6050`
* `madhephaestus/ESP32Servo`
* `arduino-libraries/Madgwick`
* `mprograms/QMC5883LCompass`
* `dfrobot/DFRobot_QMC5883`

The firmware currently implements its own Madgwick update routine as part of the attitude-estimation pipeline.

---

## Project Structure

```text
PlatformIO/
│
├── Projects/
│   └── esp 32 test/
│       │
│       ├── .vscode/
│       │
│       ├── include/
│       │
│       ├── lib/
│       │
│       ├── src/
│       │   └── main.cpp
│       │
│       ├── test/
│       │
│       ├── .gitignore
│       └── platformio.ini
│
└── README.md
```

The current ESP32 project follows the standard PlatformIO project structure, with the main firmware located in `src/main.cpp`.

---

## Getting Started

### 1. Install PlatformIO

Install PlatformIO through the PlatformIO extension for VS Code or install PlatformIO Core.

### 2. Clone the repository

```bash
git clone https://github.com/Nilajjana/PlatformIO.git
cd PlatformIO
```

### 3. Open the project

Open:

```text
Projects/esp 32 test/
```

as a PlatformIO project.

### 4. Connect the ESP32

Connect an ESP32 DevKit V1 through USB.

### 5. Configure the hardware

Connect the sensors through I²C.

The current firmware initializes:

```text
SDA → GPIO 21
SCL → GPIO 22
```

### 6. Configure Wi-Fi

Set the Wi-Fi credentials using a secure configuration method before compiling.

**Do not commit real Wi-Fi credentials to Git.**

### 7. Build and upload

Using PlatformIO:

```bash
pio run
pio run --target upload
```

The configured serial monitor speed is:

```text
115200 baud
```

You can start the monitor with:

```bash
pio device monitor
```

---

## Startup Sequence

The firmware follows approximately this sequence:

```text
ESP32 Boot
    │
    ▼
Initialize I²C
    │
    ├── Initialize QMC5883
    │
    └── Initialize MPU6050
    │
    ▼
Initialize Magnetometer Calibration
    │
    ▼
Connect to Wi-Fi
    │
    ▼
Start HTTP Server
    │
    ▼
15-second Magnetometer Calibration
    │
    ▼
Continuous Sensor Acquisition
    │
    ▼
Madgwick Orientation Estimation
    │
    ▼
Roll / Pitch / Yaw
    │
    ├── Serial output
    │
    └── HTTP telemetry
```

The actual firmware performs magnetometer calibration before entering its normal sensor-processing loop.

---

## Attitude Estimation Pipeline

The current processing pipeline is roughly:

```text
Raw Accelerometer
       │
       ▼
Low-pass Filtering
       │
       ├────────────────┐
       │                │
Raw Gyroscope      Calibrated Magnetometer
       │                │
       └───────┬────────┘
               ▼
        Madgwick Filter
               │
               ▼
          Quaternion
               │
               ▼
        Euler Conversion
               │
       ┌───────┼───────┐
       ▼       ▼       ▼
     Roll    Pitch    Yaw
```

The quaternion state is continuously normalized to maintain a valid orientation representation.

---

## Telemetry

The ESP32 prints orientation information through the serial interface:

```text
Roll: <value>° Pitch: <value>° Yaw: <value>°
```

It simultaneously provides pitch and roll through its HTTP endpoint.

This provides a simple foundation for eventually connecting the flight controller to:

* Ground-station software
* A browser dashboard
* A telemetry application
* Another embedded controller
* A future control-system interface

---

## Servo Control

The project also includes ESP32Servo as a dependency and declares multiple servo objects in the firmware.

This is intended to form part of the eventual actuator-control layer of the flight controller.

The current repository should therefore be considered a **sensor and attitude-estimation prototype**, rather than a completed closed-loop helicopter controller.

---

## Current Limitations

This project is still under development.

The current implementation does **not** represent a complete production-ready helicopter flight-control system.

Important areas still requiring development include:

* Closed-loop attitude control
* PID control
* Actuator mixing
* Robust servo control
* Sensor redundancy
* Failure detection
* Sensor fault handling
* More sophisticated calibration
* Magnetic interference compensation
* Gyroscope bias estimation
* Robust yaw estimation
* Safety mechanisms
* Watchdog / failsafe behavior
* Real-time scheduling
* Hardware-in-the-loop testing
* Extensive flight testing

Because this is flight-control software, the firmware should **not be used on an actual aircraft without extensive validation, simulation, hardware testing, and appropriate safety systems.**

---

## Project Status

The broader helicopter project is currently **on hold because of financial constraints**.

The software repository is being maintained as an engineering record and as a foundation for potentially continuing the project in the future.

The current code represents the development of the embedded sensing and attitude-estimation subsystem rather than the final flight controller.

---

## Future Development

If development resumes, the intended direction includes:

### Flight Dynamics

* Develop the helicopter control model
* Establish actuator/control-surface mapping
* Implement control-loop architecture

### Control System

* PID-based attitude stabilization
* Rate control
* Position/heading control
* Control mixing

### Sensor Fusion

* Improved IMU calibration
* Gyroscope bias estimation
* Better magnetometer calibration
* Sensor validation
* Potential redundant sensors

### Embedded Architecture

* Separate sensor, estimation, control and communication modules
* Improve real-time scheduling
* Reduce blocking operations
* Add watchdog/failsafe systems
* Improve fault handling

### Ground Station

Develop a dedicated telemetry interface capable of displaying:

```text
Roll
Pitch
Yaw
Angular Rates
Acceleration
Sensor Status
Calibration Status
System Health
```

---

## Why This Project Exists

This project is an exploration of **embedded systems, control systems, sensor fusion and real-time software**.

Rather than relying entirely on an existing flight-controller stack, the objective is to understand and build the underlying systems:

```text
Sensors
   ↓
Raw Measurements
   ↓
Calibration
   ↓
Sensor Fusion
   ↓
State Estimation
   ↓
Control Algorithm
   ↓
Actuators
```

The current repository represents the early stages of that pipeline.

---

## Disclaimer

This is an experimental engineering project.

The firmware is **not certified aviation software** and should not be treated as suitable for controlling a manned or unmanned aircraft without extensive independent validation.

Any future flight testing should be performed with appropriate safety systems, test procedures, redundancy and qualified supervision.

---

## Author

**Nilajjana**

Computer Science student interested in:

* Embedded Systems
* C/C++
* Control Systems
* Robotics
* Computer Systems
* Algorithms
* Hardware-software integration

---

## License

No explicit license has currently been specified for this repository.

If you intend for others to use, modify or distribute the project, consider adding an appropriate open-source license.
