**# AI-Driven Multi-Robot Obstacle & Collision Avoidance System**

## Project Overview
A real-time, decentralized multi-robot navigation system where **8 autonomous robots** move safely in a shared grid environment. The system utilizes a dual-layer approach: a C++ PC-based simulation for logic verification and an embedded C++ implementation for ESP32 hardware execution.

---

## Project Approach (The "Brains" of the System)
The project follows a **Decentralized Sense-Communicate-Plan-Act** cycle. Since there is no central "Master" computer, each robot is responsible for its own safety and goal-reaching.

### 1. The Decision Cycle (Every 100ms)
* **Sense:** The robot reads its current position via Odometry (Encoders) and detects local obstacles using Ultrasonic/IR sensors.
* **Communicate:** The robot broadcasts its $(X, Y)$ coordinates and velocity to all other robots using **ESP-NOW**. It simultaneously listens for the 7 other robots and updates its internal **Neighbor Table**.
* **Plan:**
    * **Global Path:** Uses **A* Algorithm** to calculate the shortest path to the goal on a configurable (5x5 / 7x7) grid.
    * **Local Coordination:** The robot checks its path against the "Neighbor Table." If another robot is in the way, it applies **Priority-Based Logic** (lower ID waits for higher ID) or reactive steering.
* **Act:** The ESP32 sends PWM signals to the **MX1508** driver to move the motors toward the next grid cell.

### 2. Development Methodology
* **Phase 1: PC Simulation (Software-in-the-Loop):** Core C++ logic is first verified on a PC using **Raylib**. This allows for debugging A* and multi-agent coordination without hardware wear and tear.
* **Phase 2: Decentralized Communication:** Implementing the ESP-NOW broadcast protocol to ensure <2ms latency data exchange between units.
* **Phase 3: Hardware Integration:** Porting the verified C++ logic to the ESP32 and tuning PID motor controllers for precise movement.

---

## System Highlights
- **Decentralized P2P Architecture:** No central master; communication via ESP-NOW Broadcast.
- **Grid-Based Navigation:** Configurable occupancy grids for structured movement.
- **Hybrid Avoidance:** A* for static obstacles + Priority rules for dynamic robot avoidance.
- **Resource Optimized:** Written in memory-safe Embedded C++ to fit within the ESP32’s 520KB SRAM.

---

## Hardware Components (Per Robot - Optimized for ₹10k Budget)
- **Controller:** ESP32 Development Board (30-pin).
- **Sensors:**
    - **HC-SR04 Ultrasonic:** Long-range obstacle detection.
    - **IR Sensors:** Short-range safety and DIY wheel encoding.
    - **MPU6050 IMU:** Heading and turn stability.
- **Actuation:** 2x TT Gear Motors + 1x Caster Wheel (Differential Drive).
- **Power:** 1x 18650 Li-ion Cell + TP4056 (Type-C Charger) + MT3608 (Boost Converter to 6V).
- **Chassis:** Lightweight Sunboard/PVC (Custom cut for agility).



---

## Software & Tools
- **Embedded Language:** C++17 (Object-Oriented).
- **Simulation:** C++/Raylib (For PC-based logic verification).
- **Communication:** ESP-NOW (Low-latency peer-to-peer).
- **IDE:** VS Code + PlatformIO.

---

## Core Software Modules
- **Odometry & State Estimation:** Tracking $(X, Y, \theta)$ using motor feedback.
- **Neighbor Table Manager:** Real-time buffer for tracking the states of the other 7 robots.
- **Pathfinder (A*):** Efficient grid-based route calculation.
- **Priority Arbiter:** Resolves deadlocks when robots meet at intersections.
- **Motor Control:** Closed-loop PWM regulation for straight-line driving and accurate 90° turns.

---
