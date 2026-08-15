# Real-Time Sensor Fusion & Conflict Resolution Engine (C++)

A high-performance, deterministic, and idempotent sensor fusion engine built for robotic vehicle indoor navigation. The system processes asynchronous, conflicting, and out-of-order sensor streams (LiDAR, Infrared, Ultrasonic), enforces a strict hierarchical conflict resolution algorithm, maintains versioned state snapshots, and supports timeline replay with audit logging.

Implemented in **Standard C++ (C++17)** with zero external dependencies and an **Arduino C++ simulation sketch**.

---

## Architecture Overview
