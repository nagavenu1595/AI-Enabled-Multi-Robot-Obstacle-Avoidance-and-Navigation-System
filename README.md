# AI-Driven Multi-Robot Obstacle & Collision Avoidance System

## Project Overview
A real-time, decentralized multi-robot navigation system where **8 autonomous robots** move safely in a shared grid environment. The system utilizes a dual-layer approach: a C++ PC-based simulation for logic verification and an embedded C++ implementation for ESP32 hardware execution.

---

## System Highlights
- **Decentralized P2P Architecture:** No central master; communication via ESP-NOW Broadcast.
- **Grid-Based Navigation:** Configurable occupancy grids with bit-packed memory optimization.
- **Software-in-the-Loop (SIL):** Shared C++ logic between PC Simulation (Raylib) and ESP32.
- **Real-Time Determinism:** Multi-threaded execution using FreeRTOS Core 0 (Comms) and Core 1 (Control).

---

## Hardware Components (Per Robot)
- **Controller:** ESP32 Development Board.
- **Sensors:** - HC-SR04 Ultrasonic (Long-range obstacle detection).
  - IR Obstacle Sensors (Close-range reactive safety).
  - **Hall Effect Encoders:** (Required for distance/grid tracking).
  - **MPU6050 IMU:** (Required for heading/orientation stability).
- **Actuation:** 4x DC Motors + DRV8833 or L298N Motor Driver.
- **Power:** 2x 18650 Li-ion Cells (7.4V) + 5V/3.3V Step-down Regulators.

---

## Software & Tools
- **Embedded Language:** C++17 (Object-Oriented).
- **Simulation:** C++/Raylib for PC-based multi-agent verification.
- **Communication:** ESP-NOW (Peer-to-Peer Protocol).
- **IDE:** VS Code + PlatformIO (preferred for C++ library management).

---

## Core Software Modules
- **Odometry & State Estimation:** Fusing encoder/IMU data for $(X, Y, \theta)$ tracking.
- **Neighbor Table Manager:** Tracking the states of 7 other robots via radio packets.
- **Global Planner:** A* or Dijkstra on a fixed 2D grid.
- **Local Planner (RVO/ORCA):** Reciprocal velocity-based collision avoidance.
- **Priority Arbiter:** ID-based deadlock resolution for multi-robot conflicts.
- **FSM Controller:** States: `IDLE`, `PLANNING`, `MOVING`, `RECOVERY`, `EMERGENCY_STOP`.
- **PID Motor Control:** Closed-loop speed regulation for precise grid alignment.
