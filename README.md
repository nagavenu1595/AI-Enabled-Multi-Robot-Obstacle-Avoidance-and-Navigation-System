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

## Key Learnings

### 1. Multi-Agent Pathfinding
Sequential planning with reservation tables prevents the exponential complexity of jointly planning for 8 robots. Robots with the longest remaining path plan first, reserving their corridors before shorter-path robots can block them.

### 2. Planning Order Matters More Than Algorithm
The single biggest source of collisions was planning order. Short-path robots planning first would reserve cells that trapped long-path robots with no exit. Sorting by descending Manhattan distance solved ~80% of conflicts before any other mechanism was needed.

### 3. Pre-Reserving Start Positions is Essential
Before any robot's A* runs, all current robot positions must be inserted into `vertexRes[t=0]`. Without this, a later-planned robot can legally route through another robot's starting cell at t=1, creating an immediate collision at step 1 that no amount of priority shuffling can fix.

### 4. Collision Detection Must Check from `fromStep`, Not Zero
During a mid-simulation replan, the historical portion of each path (steps 0 to fromStep) is already committed and may look like a collision in stale data. Checking `detectCollisions()` starting from `fromStep` instead of 0 eliminates false positives that caused infinite retry loops with no real conflict to resolve.

### 5. Three-Layer Collision Prevention Architecture
A single mechanism is not enough. The final system uses three layers that activate in order:
- **Layer 1 — Reservation Table:** A* physically cannot route through reserved cells. Handles ~95% of cases.
- **Layer 2 — Priority Shuffling:** If a conflict is detected post-plan, conflicted robots are placed at the front of the planning queue and the whole pass is retried up to `MAX_REPLAN_ATTEMPTS` times.
- **Layer 3 — Yield Resolution:** When all shuffle attempts fail, an at-goal blocker is identified and forced to temporarily step aside so the stuck robot can pass through its held cell.

### 6. Goal Hold Creates Invisible Deadlocks
Robots reserve their goal cell for `GOAL_HOLD_STEPS` timesteps after arrival to prevent other robots from walking through a "parked" robot. However, if that held cell is the only corridor to another robot's goal, the second robot becomes permanently stuck. The yield mechanism was built specifically to break this deadlock by temporarily lifting the blocker's reservations, finding a free neighbour, routing the blocker there, letting the stuck robot pass, and returning the blocker to its goal.

### 7. Obstacle-on-Robot Edge Case Requires Special Handling
When a dynamic obstacle relocates onto a robot's current cell, `validCell()` returns false for that cell, so A* returns an empty path and the robot appears stuck. The yield resolver detects `grid[curPos]==1`, queues wait-in-place moves for a few steps, and lets the next relocation event trigger a fresh replan once the obstacle has moved away.

### 8. `erasePathReservations` Must Be the Exact Inverse of `reservePath`
The yield mechanism temporarily removes a blocker's reservations to test if a stuck robot can then find a path. If the erase function does not mirror the reserve function exactly (including the goal-hold tail), stale entries remain in `vertexRes` and A* still sees the cell as blocked, making the yield attempt silently fail every time.

### 9. Smooth Replanning Requires Preserving History
When obstacles move, robots keep their movement history up to `fromStep` and only replan the future segment. Without this, robots appear to teleport back to their position at replan time, breaking the visual continuity of the simulation and making step-printed logs inconsistent.

### 10. Real-Time Performance
Full replanning of 8 robots including yield resolution completes in under 10ms even in worst-case scenarios, ensuring smooth 60fps simulation without lag spikes.

### 11. Simulation vs Hardware
PC simulation stress-tests edge cases like simultaneous goal-holds, obstacle-on-robot events, and priority deadlocks. Real ESP32 robots will additionally face wheel slip, WiFi packet loss, and motor alignment drift that simulation cannot replicate — but catching the logical edge cases in simulation saves significant hardware debugging time.

**Main Takeaway**: Deterministic, layered conflict resolution beats a single complex optimizer. Build detection before prevention, visualize every step, and treat each new edge case as a separate layer rather than patching the existing one.

---

# PENDING
- Improving software simulation  
- Hardware simulation
