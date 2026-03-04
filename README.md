# AI-Driven Multi-Robot Obstacle & Collision Avoidance System

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
- **Resource Optimized:** Written in memory-safe Embedded C++ to fit within the ESP32's 520KB SRAM.

---

## Hardware Components (Per Robot)
- **Controller:** ESP32 Development Board (30-pin).
- **Motor Driver:** **MX1508 Dual H-Bridge** (Handles high-current PWM for motors).
- **Sensors:**
    - **HC-SR04 Ultrasonic:** Long-range obstacle detection.
    - **IR Sensors:** Short-range safety and DIY wheel encoding.
    - **MPU6050 IMU:** Heading and turn stability.
- **Actuation:** 2x TT Gear Motors + 1x Caster Wheel (Differential Drive).
- **Power System:** - 1x 18650 Li-ion Cell.
    - **TP4056:** Type-C Charging/Protection module.
    - **MT3608:** Boost Converter (Stepped up to 6V for motors).
- **Chassis:** Lightweight Sunboard/PVC (Custom cut for agility).

---

## Software & Tools
- **Embedded Language:** C++17 (Object-Oriented).
- **Simulation:** C++/Raylib (For PC-based logic verification).
- **Communication:** ESP-NOW (Low-latency peer-to-peer).
- **IDE:** VS Code + Arduino.

---

## Core Software Modules
- **Odometry & State Estimation:** Tracking $(X, Y, \theta)$ using motor feedback.
- **Neighbor Table Manager:** Real-time buffer for tracking the states of the other 7 robots.
- **Pathfinder (A*):** Efficient grid-based route calculation.
- **Priority Arbiter:** Resolves deadlocks when robots meet at intersections using Robot ID hierarchy.
- **Motor Control:** Closed-loop PWM regulation via the MX1508 for straight-line driving and accurate 90° turns.

---

# COMPLETED

# Multi-Agent Path Finding (MAPF) Simulation

This project implements **Time-Expanded A\*** to generate **collision-free paths** for multiple robots moving on a grid.

---

## Collision Avoidance Features

### 1. Vertex Collision
No two robots are allowed to occupy the **same grid cell at the same timestep**.

### 2. Edge Collision
Robots are prevented from **swapping positions** in a single timestep (head-on collision prevention).

### 3. Sequential Planning
- Robots plan their paths one by one.
- Each robot considers the **already planned paths** of previously planned robots.
- Ensures cooperative, conflict-free movement.

### 4. Synchronized Cycles
All robots move **step-by-step in sync**, meaning:
- Each robot completes the current movement step before the next step begins.

---

## Obstacle Avoidance
- Cells marked **`1`** represent obstacles.
- Cells marked **`0`** represent free space.
- The pathfinding algorithm navigates only through free cells.

---

## Simulation Modes

### Mode 1: Static Obstacles
- All obstacles placed upfront before robots start moving
- 5-second countdown, then robots follow fixed paths
- Simple and predictable environment

### Mode 2: Dynamic Environment
- Obstacles appear one by one every 5 seconds
- Robots begin moving after initial 5 seconds regardless
- Continuous replanning when obstacles change positions
- After all placed, obstacles relocate round-robin every 8 seconds

### Mode 3: Random Goals + Dynamic Environment
- Robot goal assignments are randomized each run using shuffled pools
- Combines dynamic obstacle placement with non-deterministic goal layout
- Tests the planner under maximum variability

---

# Key Learnings

### 1. Multi-Agent Pathfinding Requires Structured Planning
Sequential planning with reservation tables prevents the exponential complexity of jointly planning paths for multiple robots. Each robot plans while considering already-reserved cells from earlier robots.

---

### 2. Planning Order Has Major Impact on Conflict Rates
Robots with longer Manhattan distances plan first. This prevents short-path robots from blocking narrow corridors that longer paths depend on. Ordering by path difficulty significantly reduced collision scenarios.

---

### 3. Reservation Tables Are the Core of Conflict Prevention
The reservation table tracks which robot occupies each cell at every timestep. A* is modified to reject any state that violates existing reservations, preventing vertex and edge collisions before they occur.

---

### 4. Early Collision Detection is Critical
A full simulation validation pass is run after planning. If vertex or edge conflicts are detected, the planner retries with adjusted planning order or yield mechanisms before the simulation continues.

---

### 5. Dynamic Obstacles Require Mid-Execution Replanning
When obstacles appear or relocate during execution, robots do not restart from scratch. Instead, the system preserves movement history up to the current timestep and replans only the remaining future path.

---

### 6. Replanning Must Consider All Affected Robots
When the environment changes, replanning only a single robot can introduce cascading conflicts. The final system replans all robots whose future paths intersect with the changed region.

---

### 7. Goal Holding Prevents Robots Passing Through Parked Robots
Once a robot reaches its goal, the goal cell remains reserved for several timesteps. This prevents other robots from passing through a robot that has stopped moving.

---

### 8. Yield Mechanisms Resolve Deadlocks
In rare cases where reservations create blocking situations, a robot temporarily yields by moving to a neighbouring free cell. This allows blocked robots to pass before returning to its goal.

---

### 9. Dynamic Obstacle Edge Cases Must Be Handled Explicitly
If an obstacle relocates onto a robot’s current cell, the robot temporarily waits and replans once the obstacle moves again. Without this handling, A* would fail because the current position would appear invalid.

---

### 10. Corner Cells Should Be Reserved for Robot Navigation
Obstacle placement in grid corners can create unavoidable deadlocks or block key entry/exit points for robots. Preventing obstacles from spawning in corner cells improves path availability and reduces artificial congestion.

---

### 11. Simulation is Essential for Discovering Hidden Edge Cases
The PC-based simulator exposes situations difficult to detect on hardware, such as:
- simultaneous replanning
- multi-robot corridor conflicts
- goal blocking
- obstacle-on-robot scenarios

Catching these in simulation prevents costly hardware debugging later.

---

### 12. Deterministic Layered Conflict Handling Works Best
Rather than relying on a single complex algorithm, the final system combines multiple layers:

1. Reservation-table path planning  
2. Planning-order prioritization  
3. Collision validation  
4. Dynamic replanning  
5. Yield resolution  

This layered approach produces robust behavior even in highly dynamic environments.

---

**Main Takeaway**

Reliable multi-robot coordination emerges from combining deterministic planning, reservation-based conflict prevention, and responsive replanning. Visualization and simulation are invaluable tools for discovering edge cases before deploying algorithms to real hardware systems.

---

# PENDING
- Improving software simulation  
- Hardware simulation
