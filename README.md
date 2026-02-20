# Autonomous Vision-Guided Turret

## Overview

Autonomous Vision-Guided Turret is a real-time computer vision and embedded systems project that detects, tracks, and engages colored targets using OpenCV (Python) and Arduino-based servo control.

The system integrates image processing, geometric angle mapping, serial communication, and firmware-level actuation to achieve accurate pan-tilt tracking and controlled firing using a custom-built trigger mechanism.

---

## Features

- Real-time color-based object detection using HSV segmentation
- Field-of-view (FOV) based angle mapping for accurate targeting
- Pan-tilt servo control with mechanical constraints
- Automatic firing based on target stability
- Edge and center auto-calibration system
- Manual fine-tuning of offsets during runtime
- LED and buzzer feedback for system state indication

---

## System Architecture

Camera → OpenCV Processing → Angle Mapping → Serial Communication → Arduino → Servo Actuation

---

## Communication Protocol

Python sends commands in the format:

pan,tilt,fire,aggressive

Where:
- pan – Horizontal servo angle
- tilt – Vertical servo angle
- fire – 1 to trigger shooting, 0 otherwise
- aggressive – 1 to enable automatic firing mode

---

## Project Structure
```
.
├── object_detection.py     # Real-time tracking and firing logic
├── auto_calibration.py     # Edge scanning and offset calibration
├── turret_firmware.ino     # Arduino firmware
└── README.md
```
---

## Installation

### Requirements

- Python 3.8+
- OpenCV
- NumPy
- PySerial
- Arduino IDE

Install dependencies:

pip install opencv-python numpy pyserial

Upload turret_firmware.ino to the Arduino board using Arduino IDE.

Ensure the correct serial port is configured in the Python scripts.

---

## Usage

### 1. Calibration (Required)

Run:

python auto_calibration.py

This performs:
- Mechanical edge detection
- Center alignment
- Offset estimation (OFFSET_PAN, OFFSET_TILT)

Update offsets in object_detection.py if required.

### 2. Object Tracking

Run:

python object_detection.py

The system will detect the configured color, track its centroid, adjust pan-tilt servos, and fire when stability conditions are met (if aggressive mode enabled).

---

## Runtime Controls

W/S – Adjust tilt offset  
A/D – Adjust pan offset  
Q – Reset offsets  
F – Manual fire  
Z – Toggle aggressive mode  
ESC – Exit  

---

## Mechanical Constraints

Pan range: 40° – 140°  
Tilt range: 60° – 120°  

---

## Hardware Components

- Arduino board
- 2x Servo motors (Pan, Tilt)
- 1x Servo for trigger mechanism
- USB webcam
- Laser module (for calibration)
- Buzzer
- LED
- Custom mounting structure

---

## Safety Notice

This system is intended strictly for educational and research purposes.
Do not aim at people, animals, or fragile objects.
Ensure controlled and safe testing environments.

---