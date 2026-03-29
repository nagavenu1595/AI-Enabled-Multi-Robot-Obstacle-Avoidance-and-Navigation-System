# AI-Driven Multi-Robot Obstacle & Collision Avoidance System

## Project Overview
A real-time, decentralized multi-robot navigation system where **8 autonomous robots** move safely in a shared grid environment. The system utilizes a dual-layer approach: a C++ PC-based simulation for logic verification and an embedded C++ implementation for ESP32 hardware execution.

---

## Project Approach (The "Brains" of the System)
The project follows a **Decentralized Sense-Communicate-Plan-Act** cycle. Since there is no central "Master" computer, each robot is responsible for its own safety and goal-reaching.

### 1. The Decision Cycle (Every 100ms)
- **Sense:** The robot reads its current position via Odometry (Encoders) and detects local obstacles using Ultrasonic/IR sensors.
- **Communicate:** The robot broadcasts its $(X, Y)$ coordinates and velocity to all other robots using **ESP-NOW**. It simultaneously listens for the 7 other robots and updates its internal **Neighbor Table**.
- **Plan:**
  - **Global Path:** Uses **A\* Algorithm** to calculate the shortest path to the goal on a configurable (5x5 / 7x7) grid.
  - **Local Coordination:** The robot checks its path against the Neighbor Table. If another robot is in the way, it applies **Priority-Based Logic** (lower ID waits for higher ID) or reactive steering.
- **Act:** The ESP32 sends PWM signals to the **MX1508** driver to move the motors toward the next grid cell.

### 2. Development Methodology
- **Phase 1: PC Simulation (Software-in-the-Loop):** Core C++ logic is first verified on a PC using **Raylib**. This allows for debugging A\* and multi-agent coordination without hardware wear and tear.
- **Phase 2: Decentralized Communication:** Implementing the ESP-NOW broadcast protocol to ensure <2ms latency data exchange between units.
- **Phase 3: Hardware Integration:** Porting the verified C++ logic to the ESP32 and tuning PID motor controllers for precise movement.

---

## System Highlights
- **Decentralized P2P Architecture:** No central master; communication via ESP-NOW Broadcast.
- **Grid-Based Navigation:** Configurable occupancy grids for structured movement.
- **Hybrid Avoidance:** A\* for static obstacles + Priority rules for dynamic robot avoidance.
- **Adaptive Robot Behavior:** Each robot independently adjusts speed and decision-making based on vicinity, neighbor speeds, and collision probability in real time.
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
- **Power System:**
  - 1x 18650 Li-ion Cell.
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
- **Pathfinder (A\*):** Efficient grid-based route calculation.
- **RobotBehavior Class:** Encapsulates all per-robot adaptive logic — vicinity sensing, speed adjustment, collision probability estimation, decision arbitration (PROCEED / SLOW / WAIT / DEVIATE / STEPBACK / YIELD), and smooth visual interpolation. All behavioral functions are callable from any simulation context.
- **Priority Arbiter:** Resolves deadlocks when robots meet at intersections using Robot ID hierarchy.
- **Motor Control:** Closed-loop PWM regulation via the MX1508 for straight-line driving and accurate 90° turns.

---

# COMPLETED

## Multi-Agent Path Finding (MAPF) Simulation

This project implements **Time-Expanded A\*** to generate **collision-free paths** for multiple robots moving on a grid, with a full adaptive behavior layer inspired by real-world vehicle dynamics.

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
All robots move **step-by-step in sync**, meaning each robot completes the current movement step before the next step begins.

---

## Adaptive Robot Behavior System

Each robot is modeled as a `RobotBehavior` instance that continuously senses, evaluates, and responds to its local environment. This system was designed to mirror real-world vehicle behavior — adjusting speed fluidly based on surrounding traffic, estimating collision risk ahead of time, and choosing context-appropriate maneuvers.

### Vicinity-Based Speed Adjustment
- Each robot maintains a dynamic `speedFactor` and a vicinity range (`vicMin` to `vicMax`) derived from global bounds with per-robot randomized variation.
- At every simulation instant, the robot computes the Manhattan distance to its nearest neighbor and adjusts speed using a **zoned response**: emergency slow at `vicMin`, graduated reduction through mid-zones, and a slight boost beyond `vicMax` when the path ahead is clear.
- Neighbor speed is factored in via a blending formula: if a nearby robot is slower, the current robot decelerates proportionally to closing distance; if a distant robot is faster, a small speed bonus is applied.
- Final speed is smoothed using an **Exponential Moving Average (EMA)** to eliminate abrupt jumps, then clamped within global min/max bounds.

### Collision Probability Estimation
- Each robot looks `LOOKAHEAD` steps ahead and scores potential **vertex conflicts** (same cell) and **edge conflicts** (position swaps) across all other robots.
- Closer lookahead steps are weighted more heavily. The score is normalized to produce a probability value in `[0.0, 1.0]`.

### Decision Logic
| Decision | Trigger | Behavior |
|---|---|---|
| `PROCEED` | No significant risk | Continues at current speed |
| `SLOW` | Nearest neighbor within `vicMin` | Reduces to minimum speed |
| `DEVIATE` | Collision probability ≥ `COLL_PROB_MED` | Triggers a fresh A\* re-route from current position |
| `WAIT` | Collision probability ≥ `COLL_PROB_HIGH` and lower priority | Inserts a wait step, reduces to minimum speed |
| `STEPBACK` | High risk and valid previous cell exists | Steps back one cell to clear the path for the higher-priority robot |
| `YIELD_TO_OTHER` | Cooperative replanning | A goal-sitting robot temporarily vacates to allow a blocked robot through |

### Priority Arbitration
When high collision probability is detected, robots compute a priority score based on remaining distance to goal, speed factor, and robot ID. The lower-priority robot yields via WAIT or STEPBACK; the higher-priority robot proceeds uninterrupted.

### Smooth Visual Interpolation
Each robot smoothly interpolates its rendered position between logical grid steps using a cubic ease function. Speed factor applies a small cosmetic lead or lag (clamped strictly to `[0, 1]`) so faster robots visually appear ahead of slower ones within the same step.

---

## Obstacle Avoidance
- Cells marked **`1`** represent obstacles.
- Cells marked **`0`** represent free space.
- The pathfinding algorithm navigates only through free cells.

---

## Simulation Modes

### Mode 1: Dynamic Environment
- Obstacles appear one by one on a computed interval so all `n` obstacles are placed within the first 65% of the estimated simulation window.
- Robots begin moving after an initial delay and continuously replan when the environment changes.
- After all obstacles are placed, they relocate round-robin every 12 seconds.

### Mode 2: Random Goals + Dynamic Environment
- Robot goal assignments are randomized each run using shuffled pools.
- Combines dynamic obstacle placement with non-deterministic goal layout.
- Tests the planner under maximum variability.

---

## Challenges Faced

### 1. Vertex & Edge Collisions
Two robots planning independently would target the same cell at the same timestep, or swap positions in a single step. Standard A\* has no concept of either — separate vertex and edge reservation checks had to be explicitly added.

### 2. Deadlocks
Multiple robots would form a circular waiting pattern where each robot was blocked by the next and none could move. A cooperative yield mechanism was added where a goal-sitting robot temporarily steps aside to break the deadlock.

### 3. Robot Teleportation
During replanning, stitching the new path onto the old history caused positional discontinuities — robots would visually jump to a different cell instantly when the new segment's starting cell didn't match the robot's actual current position.

### 4. Suboptimal Paths & Excessive Waiting
Robots would sit idle instead of finding spatial detours. Introducing `WAIT_PENALTY` fixed this, but tuning it was tricky — too low and robots wait unnecessarily, too high and they take extremely long detours that cost more time overall.

### 5. Replanning Cascade
Replanning one robot after an obstacle change would shift its reservations, invalidating another robot's path, which when replanned invalidated yet another — creating a cascade that a single replan pass couldn't resolve. The system was extended to detect and replan all affected robots in one pass.

### 6. Dynamic Obstacle Timer Bug
When no free cell was found for obstacle placement, `lastObsTime` was never updated, causing the placement check to fire every frame and freeze the simulation under dense obstacle conditions.

### 7. Obstacle Scheduling Bug
The original schedule computed obstacle fire times using a post-increment count, so obstacle `n` fired after most robots had already arrived. Fixed by computing fire times as `(i+1) * interval` before placement so all `n` obstacles fit within the first `OBS_END_FRAC` fraction of the simulation.

### 8. Replan Skip on Shifted Indices
`replanAll()` skipped robots whose stored path slice didn't overlap the new obstacle cell because wait/stepback insertions had shifted path indices. Fixed by introducing a `forceAll` flag that ensures every non-goal robot receives a fresh collision-free path after any obstacle event.

### 9. Visual Overshoot from Speed Factor
`visF = stepFrac * speedFactor` could exceed `1.0` for fast robots, causing a visual snap to the next cell before the logical step advanced. Fixed by making `visF` a small cosmetic lead/lag offset around `stepFrac`, strictly clamped to `[0, 1]`.

---

# Key Learnings

### 1. Multi-Agent Pathfinding Requires Structured Planning
Sequential planning with reservation tables prevents the exponential complexity of jointly planning paths for multiple robots. Each robot plans while considering already-reserved cells from earlier robots.

### 2. Planning Order Has Major Impact on Conflict Rates
Robots with longer Manhattan distances plan first. This prevents short-path robots from blocking narrow corridors that longer paths depend on. Ordering by path difficulty significantly reduced collision scenarios.

### 3. Reservation Tables Are the Core of Conflict Prevention
The reservation table tracks which robot occupies each cell at every timestep. A\* is modified to reject any state that violates existing reservations, preventing vertex and edge collisions before they occur.

### 4. Early Collision Detection is Critical
A full simulation validation pass is run after planning. If vertex or edge conflicts are detected, the planner retries with adjusted planning order or yield mechanisms before the simulation continues.

### 5. Dynamic Obstacles Require Mid-Execution Replanning
When obstacles appear or relocate during execution, robots do not restart from scratch. Instead, the system preserves movement history up to the current timestep and replans only the remaining future path.

### 6. Replanning Must Consider All Affected Robots
When the environment changes, replanning only a single robot can introduce cascading conflicts. The final system replans all robots whose future paths intersect with the changed region, using a `forceAll` flag to guarantee correctness.

### 7. Goal Holding Prevents Robots Passing Through Parked Robots
Once a robot reaches its goal, the goal cell remains reserved for several timesteps. This prevents other robots from passing through a robot that has stopped moving.

### 8. Yield Mechanisms Resolve Deadlocks
In rare cases where reservations create blocking situations, a robot temporarily yields by moving to a neighbouring free cell. This allows blocked robots to pass before the yielding robot returns to its goal.

### 9. Dynamic Obstacle Edge Cases Must Be Handled Explicitly
If an obstacle relocates onto a robot's current cell, the robot temporarily waits and replans once the obstacle moves again. Without this handling, A\* would fail because the current position would appear invalid.

### 10. Corner Cells Should Be Reserved for Robot Navigation
Obstacle placement in grid corners can create unavoidable deadlocks or block key entry/exit points for robots. Preventing obstacles from spawning in corner cells improves path availability and reduces artificial congestion.

### 11. Real-World Vehicle Dynamics Improve Coordination Quality
Modeling each robot as an adaptive agent with speed zones, EMA-smoothed velocity, and lookahead collision probability produces more natural and robust coordination than rigid wait-or-go rules. Speed adjustment based on neighbor proximity and neighbor speed mirrors how vehicles slow down, merge, and give way in real traffic — making the system both safer and more efficient.

### 12. Behavioral Decisions Should Be Encapsulated in a Single Class
Consolidating vicinity sensing, speed adjustment, collision probability estimation, priority computation, decision selection, action application, and smooth interpolation into the `RobotBehavior` class makes each behavior independently testable and composable. Any simulation mode can invoke any combination of these functions without restructuring the planner — exactly as a real vehicle's onboard controller would handle any driving scenario through a unified decision pipeline.

### 13. Simulation is Essential for Discovering Hidden Edge Cases
The PC-based simulator exposes situations difficult to detect on hardware, such as simultaneous replanning, multi-robot corridor conflicts, goal blocking, and obstacle-on-robot scenarios. Catching these in simulation prevents costly hardware debugging later.

### 14. Deterministic Layered Conflict Handling Works Best
Rather than relying on a single complex algorithm, the final system combines multiple layers:

1. Reservation-table path planning
2. Planning-order prioritization
3. Adaptive per-robot behavior (speed, vicinity, collision probability)
4. Collision validation
5. Dynamic replanning with `forceAll`
6. Yield resolution

This layered approach produces robust behavior even in highly dynamic environments.

---

**Main Takeaway**

Reliable multi-robot coordination emerges from combining deterministic planning, reservation-based conflict prevention, adaptive per-robot behavior modeled on real-world vehicle dynamics, and responsive replanning. Visualization and simulation are invaluable tools for discovering edge cases before deploying algorithms to real hardware systems.

---

# PENDING
- Improving software simulation
- Hardware simulation
