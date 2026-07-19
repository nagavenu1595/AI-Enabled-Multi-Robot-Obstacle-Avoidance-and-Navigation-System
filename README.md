# A Software Framework for Distributed Autonomous Robot Navigation

> **Decentralized multi-robot fleet navigation using Time-Expanded A\*, ESP-NOW peer-to-peer communication, and adaptive per-robot behavioral decision-making — validated in a full C++/Raylib Software-in-the-Loop simulation with zero collisions across all test runs.**

---

## Project Status

| Phase | Status |
|---|---|
| PC Software Simulation (C++/Raylib) | ✅ Complete |
| Firmware Architecture (ESP32 C++) | ✅ Complete |
| IEEE Research Paper | ✅ Drafted |
| Single Robot Hardware Assembly | 🔄 In Progress |
| 8-Robot Fleet Integration | ⏳ Pending |
| ESP-NOW Multi-Robot Field Test | ⏳ Pending |

---

## Overview

A real-time, decentralized multi-robot navigation system where **8 autonomous robots** move safely in a shared grid environment without any central coordinator. The system follows a dual-layer development approach: a C++17/Raylib PC-based Software-in-the-Loop simulation for complete logic verification, and an ESP32 firmware implementation for physical hardware execution.

Each robot independently runs the full planning and decision stack. There is no master computer issuing commands. Robots communicate exclusively peer-to-peer via **ESP-NOW** at the Wi-Fi MAC layer — no router, no access point, no external network infrastructure required — and make all navigation decisions onboard using their own sensor readings and the neighbor state table maintained through ESP-NOW broadcasts.

---

## Project Approach — Decentralized Sense-Communicate-Plan-Act Cycle

The project follows a **Decentralized Sense-Communicate-Plan-Act** cycle. Since there is no central "Master" computer, each robot is fully responsible for its own safety and goal-reaching. Every robot executes the following four-phase loop at a target period of **100ms (10Hz)**:

### 1. Sense
- Reads the **HC-SR04 ultrasonic sensor** for front obstacle detection (2–400cm range) by triggering a 10µs HIGH pulse and timing the echo return: `distance_cm = duration / 58`
- Reads **IR sensor modules** for short-range edge and side obstacle detection; LOW output state indicates obstacle within threshold
- Polls the **MPU6050 IMU** over I2C for heading correction via gyroscope Z-axis integration; accelerometer values scaled by `1/16384` g/LSB, gyroscope values by `1/131` deg·s⁻¹/LSB
- Tracks grid-cell position via wheel **odometry** using FC-03 slotted IR optocoupler pulse counts (pending hardware integration)

### 2. Communicate
- Broadcasts a **13-byte state packet** to all 7 peer robots via **ESP-NOW**
- Packet structure: Robot ID (1 byte), grid coordinates X, Y (2 bytes), speed factor (4 bytes, IEEE 754 float), behavioral decision state (1 byte), millisecond timestamp (4 bytes)
- Total payload (13 bytes) is well within the ESP-NOW maximum of 250 bytes
- **Core 0** of the dual-core ESP32 handles all ESP-NOW receive callbacks asynchronously, updating `neighborPos[]` and `robotSpeeds[]` arrays for use by the planner on Core 1
- No acknowledgment is used; the broadcast model tolerates occasional packet loss since updates arrive again within the next 100ms cycle

### 3. Plan
- **Global Path:** Time-Expanded A\* calculates a collision-free path through the (x, y, t) state space, consulting shared reservation tables `vertexRes` and `edgeRes` to avoid conflicts with already-planned robot paths
- **Conflict Resolution (Two-Layer):**
  - *Local per-step resolver:* Scans the combined path set over LOOKAHEAD+2 future steps. On detecting a vertex or edge conflict, affected robots are ranked by priority score; the lower-priority robot receives a wait step or stepback insertion
  - *Global replanner:* On any dynamic obstacle event, `replanAll(forceAll=true)` triggers simultaneous replanning for all non-goal robots, preventing the cascading reservation conflicts that arise when only the directly affected robot replans
- **Behavioral Decision:** The `RobotBehavior` class evaluates vicinity distance to the nearest neighbor, estimates collision probability over LOOKAHEAD=4 future steps, computes the robot's priority score, and selects one of six behavioral states

### 4. Act
- Selected decision maps to L298N H-bridge IN1–IN4 digital outputs
- Forward motion: IN1=HIGH, IN2=LOW (left motor), IN3=HIGH, IN4=LOW (right motor)
- Right turn: left motor forward, right motor reverse
- Left turn: left motor reverse, right motor forward
- Wait and stepback decisions: all pins LOW for one cycle
- **Core 1** of the dual-core ESP32 handles the entire Plan–Act cycle, isolated from ESP-NOW callbacks on Core 0

---

## Path Planning Algorithm — Time-Expanded A\*

Standard A\* plans over (x, y) space and cannot prevent two robots from targeting the same cell at the same time. This system extends A\* to search over **(x, y, t)** state space, where each search node represents the robot's position at a specific timestep. This allows the planner to reject any state that conflicts with reservations left by previously planned robots.

### Evaluation Function

```
f(n) = g(n) + h(n)
h(n) = |x - x_goal| + |y - y_goal|     (admissible Manhattan distance heuristic)
```

`g(n)` is the accumulated cost from the start state. Wait actions carry an additive `WAIT_PENALTY` to discourage unnecessary idling. Revisiting a cell within a `REVISIT_WINDOW` of recent ancestors carries a `BACKTRACK_PENALTY` to prevent oscillation.

### Reservation Checks

Before accepting any successor state `(x', y', t+1)` the planner enforces:

```
Vertex check : (x', y') ∉ vertexRes[t+1]
Edge check   : ((x',y') → (x,y)) ∉ edgeRes[t+1]
```

The goal cell is accepted only if `goalCanSettle(g, t)` is satisfied — meaning no other robot has reserved the goal at or after timestep t up to the planning horizon.

### Planning Order

Robots are planned in order of **decreasing Manhattan distance to goal**: robots with longer paths plan first. This prevents short-path robots from occupying narrow corridors that longer paths depend on. After each robot plans, its path is registered into the reservation tables via `reservePath()` before the next robot begins. The planning order itself is randomized across replan attempts to escape ordering-induced deadlocks.

---

## Conflict Resolution

### Local Per-Step Resolver

After all robots complete initial planning, `detectCollisions()` validates the combined path set over a LOOKAHEAD+2 window. On detecting a vertex or edge conflict, affected robots are ranked by a composite priority score:

```
P_i = -d_i + 0.5 * v_i + 0.01 * i
```

Where `d_i` is the remaining Manhattan distance to goal (negative — robots closer to goal have higher priority), `v_i` is the current speed factor, and `i` is the robot index serving as a tiebreaker. The lower-priority robot receives a wait step or stepback insertion; the higher-priority robot proceeds uninterrupted.

### Global Replanner

On any dynamic obstacle event, `replanAll()` is invoked with `forceAll=true`, causing every non-goal robot to replan simultaneously from its current position. Up to `MAX_REPLAN_ATTEMPTS=5` iterations with randomized planning order permutations are attempted before falling back to a cooperative yield pass and, if necessary, an extended time-horizon search with doubled `MAX_TIME`. This single-pass global replan prevents the cascading conflicts that arise when only the directly affected robot replans and its shifted reservations invalidate a neighbor's path.

---

## Adaptive Robot Behavior System

Each robot is modeled as a `RobotBehavior` instance that continuously senses, evaluates, and responds to its local environment. The system was designed to mirror real-world vehicle behavior — adjusting speed fluidly based on surrounding traffic, estimating collision risk ahead of time, and choosing context-appropriate maneuvers.

### Vicinity-Based Speed Adjustment

Each robot maintains a dynamic `speedFactor` and a vicinity range (`vicMin` to `vicMax`) derived from global bounds with per-robot randomized variation. At every simulation instant, the robot computes the Manhattan distance to its nearest neighbor and adjusts speed using a **zoned response**:

- Distance ≤ `vicMin`: emergency slow, speed drops to `SPEED_MIN = 0.30`
- Distance = `vicMin+1`: floor 0.35, gradual ramp
- Distance ≤ `vicMax/2`: floor 0.55, continued ramp
- Distance ≤ `vicMax`: approaching base speed, slight blend with neighbor speed
- Distance > `vicMax`: slight boost up to `SPEED_MAX = 1.50`

Neighbor speed is factored in via a blending formula: if a nearby robot is slower, the current robot decelerates proportionally to closing distance; if a distant robot is faster, a small speed bonus is applied. Final speed is smoothed using an **Exponential Moving Average (EMA, α = 0.65)** to eliminate abrupt jumps, then clamped within [SPEED_MIN, SPEED_MAX]:

```
v_t = 0.65 * v_raw + 0.35 * v_(t-1)
```

### Collision Probability Estimation

Each robot looks `LOOKAHEAD=4` steps ahead and scores potential conflicts across all other robots:

```
P_coll = min(1.0, Σ(w_k^vertex + w_k^edge) / ((N-1) * L * 0.80))
```

Vertex conflict weights: d=0 → 0.80, d=1 → 0.30, d=2 → 0.10. Edge (head-on swap) weight: 0.60. Closer lookahead steps are weighted more heavily. The score is normalized to produce a probability value in [0.0, 1.0].

### Decision State Machine

| Decision | Trigger | Behavior |
|---|---|---|
| `PROCEED` | No significant risk | Continues at current speed |
| `SLOW` | Nearest neighbor within `vicMin` | Reduces to minimum speed |
| `DEVIATE` | Collision probability ≥ `COLL_PROB_MED` (0.40) | Triggers a fresh A\* re-route from current position |
| `WAIT` | Collision probability ≥ `COLL_PROB_HIGH` (0.70) and lower priority | Inserts a wait step, reduces to minimum speed |
| `STEPBACK` | High risk and valid previous cell exists | Steps back one cell to clear the path for the higher-priority robot |
| `YIELD_TO_OTHER` | Cooperative replanning | A goal-sitting robot temporarily vacates to allow a blocked robot through |

### Cooperative Yielding

When a robot cannot find a path of length greater than one cell despite multiple replan attempts, `coopYield` identifies any goal-sitting robot whose goal cell lies on or near the blocked robot's required corridor. The goal-sitting robot temporarily moves to an adjacent free cell and waits for `YIELD_WAIT_STEPS` timesteps before returning, clearing the reservation long enough for the blocked robot to pass.

### Smooth Visual Interpolation

Each robot smoothly interpolates its rendered position between logical grid steps using a **cubic ease function** `ease = visF² * (3 - 2*visF)`. Speed factor applies a small cosmetic lead or lag offset strictly clamped to [0, 1], so faster robots visually appear ahead of slower ones within the same step without overshooting to the next cell before the logical step advances.

---

## Simulation — C++17 / Raylib (✅ Complete)

The full 8-robot system was developed and validated in a PC-based Software-in-the-Loop (SIL) simulation before any hardware work. The simulation renders all eight robots simultaneously with smooth cubic-interpolated motion and supports two operating modes.

### Mode 1: Dynamic Environment
- Obstacles appear one by one on a computed interval so all `n` obstacles are placed within the first 65% of the estimated simulation window (`OBS_END_FRAC = 0.65`)
- Obstacle interval: `max(2.5s, (EST_SIM_STEPS * TPS * OBS_END_FRAC) / n)`
- Robots begin moving after `SIM_START_DELAY = 4.0s` and continuously replan when the environment changes
- After all obstacles are placed, they relocate round-robin every 12 seconds via `tryRelocObs()`

### Mode 2: Random Goals + Dynamic Environment
- Robot goal assignments are randomized each run using shuffled pools (4 row-edge goals + 4 column-edge goals)
- Uniqueness of goal assignment verified before simulation start
- Combines dynamic obstacle placement with non-deterministic goal layout
- Tests the planner under maximum variability

### Simulation Results (10 runs, 5 per mode)

| Metric | Mode 1 | Mode 2 |
|---|---|---|
| Vertex collisions | 0 | 0 |
| Edge collisions | 0 | 0 |
| Goal completion rate | 100% | 100% |
| Deadlocks resolved | All | All |
| Dynamic obstacle replanning | Every run | Every run |

---

## Obstacles Avoidance

- Cells marked `1` represent obstacles (static or dynamically placed)
- Cells marked `0` represent free space
- A\* navigates only through free cells; obstacle cells are rejected as invalid successors
- When an obstacle is placed on a robot's future path, `replanAll(forceAll=true)` is immediately triggered
- When an obstacle is placed on a robot's current cell, the `startOnObs` flag allows the robot to plan escape moves from its blocked position

---

## Hardware Design — Single Robot Unit

### Components

| Component | Part | Purpose |
|---|---|---|
| Controller | ESP32 DevKit V1 (30-pin, dual-core Xtensa LX6 @ 240MHz) | Main processor + ESP-NOW |
| Motor Driver | L298N Dual H-Bridge | Motor direction + PWM control |
| Motors | 2× TT Gear Motor (7–9V, 1:48 ratio, 65mm wheels) | Differential drive |
| Ultrasonic | HC-SR04 | Front obstacle detection (2–400cm) |
| IR Sensors | 2× IR Module | Side/edge obstacle detection |
| IMU | MPU6050 (I2C, addr 0x68) | Heading correction + tilt sensing |
| Encoder | FC-03 Slotted IR Optocoupler | Wheel odometry (pending integration) |
| Power Cell | 18650 Li-ion (2200mAh) | Primary power source |
| Charger | TP4056 Type-C with DW01 protection | Safe charging + over-discharge protection |
| Boost | MT3608 DC-DC Boost Converter | Steps up to 8V for motor rail |
| Chassis | Sunboard/PVC foam, two-layer, 20×15cm | Lightweight frame + electronics mount |
| Caster | 360° ball caster | Third contact point |

### GPIO Pin Assignment

| GPIO | Board Label | Connected To | Signal Type |
|---|---|---|---|
| GPIO 25 | D25 | L298N IN1 (Left forward) | Digital output |
| GPIO 26 | D26 | L298N IN2 (Left backward) | Digital output |
| GPIO 27 | D27 | L298N IN3 (Right forward) | Digital output |
| GPIO 14 | D14 | L298N IN4 (Right backward) | Digital output |
| GPIO 12 | D12 | L298N ENA (Left PWM) | PWM output |
| GPIO 13 | D13 | L298N ENB (Right PWM) | PWM output |
| GPIO 5  | D5  | HC-SR04 TRIG | Digital output |
| GPIO 18 | D18 | HC-SR04 ECHO (via voltage divider) | Digital input |
| GPIO 34 | D34 | IR Sensor Left OUT | Digital input (input-only pin) |
| GPIO 35 | D35 | IR Sensor Right OUT | Digital input (input-only pin) |
| GPIO 21 | D21 | MPU6050 SDA | I2C data |
| GPIO 22 | D22 | MPU6050 SCL | I2C clock |

### Power Architecture

```
18650 Battery
    │
    ├──→ TP4056 B+ / B−     (charging + over-discharge protection)
    │         │
    │     OUT+ / OUT−
    │         │
    │     MT3608 IN+/IN−    (boost converter)
    │         │
    │     OUT+ (8V) ─────→ L298N 12V pin (motor power rail)
    │                  ─────→ 100µF decoupling capacitor
    │
    └──→ L298N 5V pin ─────→ ESP32 VIN (logic power, 5V regulated)
                                │
                            ESP32 3V3 pin ─→ HC-SR04 VCC
                                          ─→ IR Sensor VCC
                                          ─→ MPU6050 VCC
```

### Critical Wiring Requirements

- **HC-SR04 ECHO voltage divider:** ECHO pin outputs 5V; ESP32 GPIO maximum is 3.3V. Voltage divider (1kΩ series + 2kΩ shunt to GND) reduces signal to `5V × (2/3) ≈ 3.33V`. Skipping this will damage GPIO 18 over time
- **MPU6050 I2C pull-ups:** 4.7kΩ pull-up resistors required on both SDA and SCL lines to 3.3V for reliable I2C communication
- **MPU6050 AD0 → GND:** Sets I2C address to 0x68; leave floating or pull HIGH for 0x69
- **Decoupling capacitors:** 100µF across L298N motor supply pins absorbs switching transients; 100nF at ESP32 VCC pin suppresses RF noise from Wi-Fi transmitter
- **Separate power rails:** Motor power (8V, MT3608) and logic power (5V, L298N regulator) must be on separate rails to prevent motor switching transients from resetting the ESP32

---

## Hardware Assembly Status (🔄 In Progress)

| Component | Status | Notes |
|---|---|---|
| Chassis (two-layer sunboard) | 🔄 In progress | Two-layer design with standoffs |
| Motors + wheels mounted | ✅ Done | Differential drive verified |
| L298N wired + tested | ✅ Done | Motor forward/backward/turn verified via USB |
| TP4056 + MT3608 power chain | ✅ Done | 8V output verified on multimeter |
| HC-SR04 with voltage divider | ✅ Done | Distance readings in Serial Monitor ✅ |
| IR Sensor (Left) | ✅ Done | Obstacle detection verified ✅ |
| IR Sensor (Right) | ❌ Faulty unit | Replacement pending |
| MPU6050 | ⏳ Pending | Header pins need soldering |
| ESP32 standalone battery power | 🔄 Debugging | L298N 5V → ESP32 VIN wiring in progress |
| Full integration test | ⏳ Pending | Awaiting battery power resolution |

**Individual component tests completed on physical hardware:**
- Both motors spin correctly to forward/backward/turn commands via USB ✅
- HC-SR04 returns consistent distance readings in Serial Monitor ✅
- Left IR sensor correctly outputs OBSTACLE / Clear states ✅

---

## Firmware Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ESP32 DevKit V1                         │
│                                                             │
│  Core 0 (ESP-NOW Handler)    Core 1 (Main Loop — 100ms)    │
│  ──────────────────────      ─────────────────────────────  │
│  ESP-NOW receive callback →  Sense                          │
│  Update neighborPos[]            HC-SR04, IR, MPU6050       │
│  Update robotSpeeds[]        Communicate                    │
│                                  Broadcast 13-byte packet   │
│                              Plan                           │
│                                  timeExpandedAStar()        │
│                                  RobotBehavior::tick()      │
│                                  resolveConflicts()         │
│                              Act                            │
│                                  L298N GPIO signals         │
└─────────────────────────────────────────────────────────────┘
```

**Target cycle time:** 100ms | **Logical grid step (TPS):** 1.4 seconds

---

## Software & Tools

| Layer | Technology |
|---|---|
| Simulation language | C++17 (Object-Oriented) |
| Simulation graphics | Raylib (real-time 2D rendering) |
| Firmware language | Arduino C++ (ESP32) |
| Communication protocol | ESP-NOW (Wi-Fi MAC layer, no router) |
| IDE | VS Code + Arduino IDE |
| Research paper | LaTeX (IEEE conference format) |

---

## Core Software Modules

| Module | Description |
|---|---|
| `timeExpandedAStar()` | (x,y,t) state space search with vertex/edge reservation checks, goal-settle validation, and BACKTRACK_PENALTY |
| `resolveConflicts()` | Per-step local conflict detector + behavioral decision applier for all robots |
| `replanAll()` | Global replanner with forceAll flag, randomized ordering, cooperative yield fallback, and extended horizon search |
| `RobotBehavior` | Per-robot class encapsulating vicinity sensing, EMA speed control, collision probability estimation, priority computation, decision arbitration, and smooth visual interpolation |
| `detectCollisions()` | Validates full path set for vertex and edge conflicts over configurable future window |
| `reservePath()` / `erasePathReservations()` | Adds/removes a robot's path from vertex and edge reservation tables |
| `coopYield()` / `resolveYield()` | Cooperative deadlock resolution — goal-sitting robot temporarily vacates to allow blocked robot through |
| `assembleAndReserve()` | Stitches history + new path segment into full path; rebuilds all reservations from scratch to prevent accumulation errors |

---

## Challenges Faced

### 1. Vertex and Edge Collisions
Two robots planning independently would target the same cell at the same timestep, or swap positions in a single step. Standard A\* has no concept of either. Separate vertex and edge reservation checks were explicitly added to the A\* successor validation, and a full `detectCollisions()` pass runs after planning to catch any remaining conflicts.

### 2. Deadlocks
Multiple robots formed circular waiting patterns where each robot was blocked by the next and none could move. A cooperative yield mechanism was added: `coopYield` identifies a goal-sitting robot blocking the corridor, moves it to an adjacent free cell for `YIELD_WAIT_STEPS`, then allows the blocked robot to pass before the yielding robot returns.

### 3. Robot Teleportation During Replanning
Stitching a new path segment onto the old history caused positional discontinuities — robots would visually jump to a different cell instantly when the replanned segment's starting cell didn't match the robot's actual current position. Fixed in `assembleAndReserve()` by explicitly aligning the segment start to the current position before stitching.

### 4. Suboptimal Paths and Excessive Waiting
Robots would sit idle instead of finding spatial detours. Introducing `WAIT_PENALTY` fixed this, but tuning required care — too low and robots wait unnecessarily, too high and they take extremely long detours that cost more overall time.

### 5. Replanning Cascade
Replanning one robot after an obstacle change shifted its reservations, invalidating a neighbor's path. That neighbor's replan invalidated another — creating a cascade that a single replan pass couldn't resolve. Fixed by `replanAll(forceAll=true)`, which replans all non-goal robots simultaneously in one pass.

### 6. Dynamic Obstacle Timer Freeze
When no free cell was found for obstacle placement, `lastObsTime` was never updated, causing the placement check to fire every frame and freeze the simulation under dense obstacle conditions. Fixed by always updating the timer regardless of placement success.

### 7. Obstacle Scheduling Bug
The original schedule computed obstacle fire times using a post-increment count, so obstacle `n` fired after most robots had already arrived. Fixed by computing fire times as `(i+1) * interval` before placement so all `n` obstacles fit within the first `OBS_END_FRAC` fraction of the simulation window.

### 8. Replan Skip on Shifted Path Indices
`replanAll()` skipped robots whose stored path slice didn't overlap the new obstacle cell because wait/stepback insertions had shifted path indices. Fixed by introducing the `forceAll` flag that ensures every non-goal robot receives a fresh collision-free path after any obstacle event, regardless of index state.

### 9. Visual Overshoot from Speed Factor
`visF = stepFrac * speedFactor` could exceed 1.0 for fast robots, causing a visual snap to the next cell before the logical step advanced. Fixed by making `visF` a small cosmetic lead/lag offset around `stepFrac`, strictly clamped to [0, 1], so the rendered position never crosses into the next cell early.

---

## Key Learnings

### 1. Multi-Agent Pathfinding Requires Structured Planning
Sequential planning with reservation tables prevents the exponential complexity of jointly planning paths for multiple robots. Each robot plans while considering already-reserved cells from earlier robots.

### 2. Planning Order Has Major Impact on Conflict Rates
Robots with longer Manhattan distances plan first. This prevents short-path robots from blocking narrow corridors that longer paths depend on. Ordering by path difficulty significantly reduced collision scenarios.

### 3. Reservation Tables Are the Core of Conflict Prevention
The reservation table tracks which robot occupies each cell at every timestep. A\* is modified to reject any state that violates existing reservations, preventing vertex and edge collisions before they occur.

### 4. Early Collision Detection Is Critical
A full simulation validation pass runs after planning. If vertex or edge conflicts are detected, the planner retries with adjusted planning order or yield mechanisms before the simulation continues.

### 5. Dynamic Obstacles Require Mid-Execution Replanning
When obstacles appear or relocate during execution, robots do not restart from scratch. The system preserves movement history up to the current timestep and replans only the remaining future path.

### 6. Replanning Must Consider All Affected Robots
When the environment changes, replanning only a single robot can introduce cascading conflicts. The final system replans all robots whose future paths intersect with the changed region using a `forceAll` flag.

### 7. Goal Holding Prevents Robots Passing Through Parked Robots
Once a robot reaches its goal, the goal cell remains reserved for `GOAL_HOLD_STEPS` timesteps. This prevents other robots from passing through a robot that has stopped moving.

### 8. Yield Mechanisms Resolve Deadlocks
In rare cases where reservations create circular blocking, a robot temporarily yields by moving to a neighboring free cell. This allows blocked robots to pass before the yielding robot returns to its goal.

### 9. Dynamic Obstacle Edge Cases Must Be Handled Explicitly
If an obstacle relocates onto a robot's current cell, the robot temporarily waits and replans once the obstacle moves. Without the `startOnObs` flag, A\* would fail because the current position would appear invalid.

### 10. Corner Cells Should Be Reserved for Robot Navigation
Obstacle placement in grid corners can create unavoidable deadlocks or block key entry/exit points. Preventing obstacles from spawning in corner cells improves path availability and reduces artificial congestion.

### 11. Real-World Vehicle Dynamics Improve Coordination Quality
Modeling each robot as an adaptive agent with speed zones, EMA-smoothed velocity, and lookahead collision probability produces more natural and robust coordination than rigid wait-or-go rules. Speed adjustment based on neighbor proximity and neighbor speed mirrors how vehicles slow down, merge, and give way in real traffic.

### 12. Behavioral Decisions Should Be Encapsulated in a Single Class
Consolidating vicinity sensing, speed adjustment, collision probability estimation, priority computation, decision selection, action application, and smooth interpolation into the `RobotBehavior` class makes each behavior independently testable and composable. Any simulation mode can invoke any combination of these functions without restructuring the planner.

### 13. Simulation Is Essential for Discovering Hidden Edge Cases
The PC-based simulator exposed situations extremely difficult to detect on hardware: simultaneous replanning, multi-robot corridor conflicts, goal blocking, and obstacle-on-robot scenarios. Nine critical bugs were identified and fixed in simulation before any physical hardware was assembled.

### 14. Deterministic Layered Conflict Handling Works Best
Rather than relying on a single complex algorithm, the final system combines multiple independent layers:
1. Reservation-table path planning
2. Planning-order prioritization by path difficulty
3. Adaptive per-robot behavior (speed zones, EMA, collision probability)
4. Per-step collision validation pass
5. Dynamic replanning with `forceAll`
6. Cooperative yield resolution

This layered approach produces robust behavior even in highly dynamic environments where any single layer would fail alone.

### 15. Separate Power Rails Are Non-Negotiable in Hardware
Motor switching transients on a shared power rail reset the microcontroller unpredictably. Separating motor power (8V, MT3608) from logic power (5V, L298N regulator → ESP32 VIN) eliminated this class of hardware failure entirely.

---

## Research Paper

**Title:** *ESP-NOW Based Multi-Robot Fleet Navigation with Collision Avoidance*

IEEE conference format paper documenting the system architecture, planning algorithm, firmware design, hardware design, simulation results, and theoretical physical performance analysis. Pending final simulation metric logging and arXiv submission.

---

## Pending Work

- [ ] Complete single robot hardware assembly and standalone battery power test
- [ ] Solder MPU6050 header pins and verify I2C communication on hardware
- [ ] Replace faulty right IR sensor
- [ ] Run full A\* navigation test on physical robot over marked 6×6 grid
- [ ] Assemble and integrate 8-robot fleet
- [ ] ESP-NOW field test with all 8 robots at scale
- [ ] Encoder-based closed-loop odometry integration (FC-03 slotted IR)
- [ ] Battery runtime characterization under continuous multi-robot operation
- [ ] Finalize simulation metric table and submit research paper to arXiv

---

**Main Takeaway**

Reliable multi-robot coordination emerges from combining deterministic planning, reservation-based conflict prevention, adaptive per-robot behavior modeled on real-world vehicle dynamics, and responsive replanning. Simulation is invaluable for discovering edge cases before deploying algorithms to real hardware — nine bugs found and fixed in simulation saved significant hardware debugging time and component risk.

---
