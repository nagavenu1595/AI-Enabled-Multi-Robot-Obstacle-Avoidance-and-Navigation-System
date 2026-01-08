# AI-Driven Multi-Robot Obstacle & Collision Avoidance System

## Project Overview
A real-time multi-robot navigation system where autonomous robots move safely in a shared grid environment by avoiding static obstacles and dynamic collisions. The system is implemented fully in embedded C++ on ESP32, ensuring real-time performance within memory constraints.

---

## System Highlights
- Decentralized control architecture  
- Grid-based navigation (5×5 / 7×7 configurable)  
- Real-time obstacle and collision avoidance  
- Memory-safe embedded implementation  

---

## Hardware Components
- ESP32 Development Board  
- Ultrasonic Sensor (HC-SR04) – distance & obstacle detection  
- IR Obstacle Sensor (Digital) – short-range detection  
- DC Motors (4 per robot)  
- Motor Driver (L298N / L293D)  
- Li-ion Battery Pack (7.4V / 18650 cells)  

---

## Software & Tools
- **Programming Language:** Embedded C++  
- **Framework:** Arduino (ESP32)  
- **Development Tools:** Arduino IDE / VS Code  

---

## Core Software Modules
- **Sensor Interface:** Ultrasonic & IR data acquisition  
- **Grid Mapping:** Fixed-size 2D grid representation  
- **Robot State Management:** Position, direction, destination  
- **Navigation Logic:** Rule-based movement strategy  
- **Obstacle Avoidance:** Reactive directional re-planning  
- **Collision Avoidance:** Distance-based robot coordination  
- **Finite State Machine (FSM):** Deterministic control flow  
- **Motor Control:** PWM-based speed and direction control  
- **Communication:** ESP-NOW / Wi-Fi for robot coordination  

---
