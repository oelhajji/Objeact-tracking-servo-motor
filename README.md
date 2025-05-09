# Raspberry Pi Object Tracking with Arduino-Controlled Servos

This project combines **Raspberry Pi**, **OpenCV**, **PiCamera**, and **Arduino** to create a real-time object tracking system using color detection. The Raspberry Pi processes visual data, tracks a colored object, and sends control signals to servos connected to an Arduino to adjust their orientation accordingly.

![Demo](Simulation.gif)

---

## Features

- Real-time video capture and processing using PiCamera
- HSV-based color detection with OpenCV
- Centroid tracking and proportional control
- Serial communication between Raspberry Pi and Arduino
- Dual-servo control for horizontal and vertical movement

---

## Hardware Requirements

- Raspberry Pi (tested on Raspberry Pi 4)
- PiCamera module
- Arduino Uno (or compatible)
- 2x Servo Motors (SG90 or similar)
- Jumper wires
- Breadboard (optional for wiring)
- Object with a distinctive color (e.g., blue ball)

---

## Software Requirements

### On Raspberry Pi:

- Python 3
- OpenCV
- NumPy
- `pyserial`
- `picamera2`

Install dependencies:
```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-pip
pip install pyserial picamera2

