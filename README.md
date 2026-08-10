# Smart Barbell Tracker

A professional-grade hardware and software system for **Velocity Based Training (VBT)** and Bar Symmetry analysis in Powerlifting (Squat, Bench Press, Deadlift).

## Project Overview

Smart Barbell Tracker utilizes an **ESP32-C6 microcontroller** paired with an **MPU6500 (or ISM330DHCX) IMU sensor**, mounted on a 50mm Olympic barbell sleeve using a 3D-printed collar. It accurately measures real-time vertical acceleration, velocity, 2D bar trajectory, and tilt symmetry (using dual sensors) to provide elite-level analytical feedback for powerlifters.

## Features

- **VBT Analytics**: Automatically tracks Mean Concentric Velocity (MCV), Peak Velocity, Range of Motion (ROM), and identifies the Sticking Point (lowest acceleration).
- **Fatigue Monitoring**: Compares the first and last repetition of a set. Triggers audio/visual warnings if velocity loss exceeds a user-defined threshold, preventing muscle failure and optimizing the training stimulus.
- **Symmetry & Bar Tilt**: Dual-sensor mode compares the left and right sides of the barbell.
- **Advanced UI (PyQt6 & PyQtGraph)**: Dark-themed, GPU-accelerated interface with real-time plotting, interactive crosshairs, custom boundaries, and zoom capabilities.
- **Training Diary**: Logs all sets securely in a gzip-compressed JSON file, displaying history via an interactive calendar.
- **IPF GL Calculator**: Built-in calculator for IPF points, helping track competitive performance.
- **Bilingual Interface**: Full support for English (EN) and Slovak (SK).

## Hardware & Firmware Requirements

The Python application communicates with an ESP32-C6 (or similar BLE 5.3 capable microcontroller) over Bluetooth Low Energy. 

**Firmware requirements:**
1. **Device Name**: `Smart_Collar_ESP32`
2. **Characteristic UUID**: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
3. **Data Stream format**: `acc_L,acc_R,real_dt\n`
   - `acc_L`, `acc_R`: Pure linear acceleration without gravity (using a Complementary filter `alpha = 0.98`).
   - `real_dt`: Delta time in seconds between samples.
4. **BLE Commands (App to ESP32)**:
   - `C` -> Gyroscope calibration in resting state. MCU replies with `CALIB_DONE\n`.
   - `A` -> 6-point accelerometer calibration. MCU stores factors in NVS memory and replies with `ACCEL_CALIB_DONE\n`.
   - `B` -> Battery request. MCU replies with `BAT:{percent},{voltage}V\n`.

*Note: The hardware specifically avoids 9-DOF Magnetometers (MPU9250) due to severe Hard/Soft Iron distortion caused by steel plates, barbells, and squat racks.*

## Setup & Installation

**Prerequisites:**
- Python 3.10 or higher.
- A compatible ESP32 module with the respective firmware flashed.

1 **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Run the application:**
   ```bash
   python beta1.0.py
   ```

## License
MIT License
