# robot-sim-26
Autonomous Robotics System designed to compete against other Robotics in a game

## Overview

This project involves developing software for an autonomous, mobile and functional robot, combinign real-time sensor data, movement control, and localisation algorithms to allow the robot to navigate its environment, avoid stationary and mobile objects, and interact with game items.

This system uses a modular architecture, seperating robot control, hardware drivers, navigation processes, sensor utilities and localisation algorithms

## Technology Stack

Python - Core robotics software
NumPy - Numerical computation and mathematical operations
Unscented Kalman Filter / Pose Estimation - Robot localisation and state estimation
Computer Vision - Marker Detection and navigation
Ultrasonic Sensorts - Distance and proximity measurement
Robotics Hardware APIs - Motor, servo and sensor integration

## Technical Features
- Autonomous Navigation
- Computer Vision
- Sensor Integration
- Robot Localisation
- Hardware Control

## Project Structure

robot-sim-26/
│
├── robot.py
│
├── challenges/
│   ├── movement.py
│   ├── sensing.py
│   ├── stopping.py
│   ├── vision_angle.py
│   └── vision_distance.py
│
└── src/
    ├── main.py
    ├── process.py
    ├── utils.py
    ├── constants.py
    │
    ├── drivers/
    │   ├── motor.py
    │   ├── servo.py
    │   └── ultrasound.py
    │
    └── filter/
        ├── kf_utils.py
        ├── localiser.py
        └── ukf.py

## Contributors

Collaborative project. Individual contributions should be credited according to the work undertaken by each team member.