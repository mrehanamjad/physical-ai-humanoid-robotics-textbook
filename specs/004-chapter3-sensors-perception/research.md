---
title: "Research Log: Chapter 3 – Sensors and Perception Basics"
feature: "004-chapter3-sensors-perception"
status: "draft"
date: "2025-12-27"
---

## 1. Research Questions

This document tracks the research needed to fulfill the requirements of `plan.md` for Chapter 3.

### Foundational Research (Phase 1)

1.  **Intuitive Explanations**: What are the clearest, most intuitive online resources (articles, videos, course notes) for explaining the fundamental operating principles of:
    - RGB Cameras (pinhole model)
    - Depth Cameras (Structured Light vs. Time of Flight)
    - 2D/3D LiDAR (Time of Flight, rotating mirror assembly)
    - IMUs (MEMS accelerometers and gyroscopes, concept of drift)

2.  **Canonical Use Cases**: For each sensor, what are the canonical tasks it is used for in humanoid robotics?
    - *Camera*: Object recognition, person tracking, reading signs.
    - *LiDAR*: Obstacle avoidance, SLAM (Simultaneous Localization and Mapping).
    - *IMU*: Balance and stabilization, state estimation.

3.  **Strengths & Weaknesses**: What are the most commonly cited trade-offs for each sensor modality in robotics literature?
    - *Example Keyword*: "LiDAR vs camera robotics", "IMU drift problem".

### Just-in-Time Research (Phase 2)

- **Proprioceptive vs. Exteroceptive**: Find formal definitions and clear examples to create a solid distinction.
- **Data Formats (Conceptual)**: How to describe the output of each sensor without getting into specific data structures? (e.g., "A grid of color values" for a camera, "A cloud of 3D points" for LiDAR).
- **Sensor Fusion**: Find a simple, compelling example of sensor fusion that highlights its benefits (e.g., fusing IMU and wheel encoders for better odometry, or LiDAR and camera for robust object detection).
- **Noise**: What are common, easy-to-understand examples of noise for each sensor? (e.g., camera grain in low light, LiDAR "ghost points" from reflective surfaces, IMU drift during constant motion).

## 2. Research Findings

*(This section will be populated as research is conducted.)*

### Sensor Principles & Use Cases

*   **Cameras (RGB, Depth)**:
    *   **Principle**: RGB cameras use a pinhole model to capture color images. Depth cameras use structured light (projecting a known pattern) or Time of Flight (measuring light travel time) to determine distance.
    *   **Use Cases**: Object recognition, facial recognition, reading text, visual navigation (vSLAM). Depth is crucial for grasping and close-range obstacle avoidance.
    *   **Strengths**: High-resolution, rich color information, inexpensive (RGB).
    *   **Weaknesses**: Susceptible to lighting conditions, can be computationally expensive to process, provides 2D information only (RGB).

*   **LiDAR (2D, 3D)**:
    *   **Principle**: Works on Time of Flight (ToF) of laser beams. 2D LiDARs use a single rotating laser, while 3D LiDARs use multiple lasers or a rotating mirror assembly to create a 3D point cloud.
    *   **Use Cases**: SLAM, long-range obstacle detection, environment mapping.
    *   **Strengths**: Accurate depth measurement, works in various lighting conditions (including darkness), long-range capabilities.
    *   **Weaknesses**: Provides no color information, can be expensive, struggles with reflective or transparent surfaces.

*   **IMUs (Inertial Measurement Units)**:
    *   **Principle**: Combine accelerometers (measure linear acceleration) and gyroscopes (measure angular velocity), typically using MEMS technology.
    *   **Use Cases**: Balance and stability control, orientation tracking, state estimation.
    *   **Strengths**: Provide high-frequency motion data, small, low power consumption.
    *   **Weaknesses**: Suffer from drift over time (accumulation of small errors), requiring fusion with other sensors for accurate long-term localization.

### Decision: Presentation Order - Sensors → Data → Perception

- **Rationale**: A survey of leading robotics courses and textbooks [1, 2] shows that a bottom-up approach is standard pedagogical practice. It allows learners to build a mental model from the physical sensor to the abstract information used by the robot's brain. This aligns with the spec's focus on foundational understanding.
- **Alternatives Considered**:
    - **Task-Driven**: Starting with a task (e.g., "avoiding obstacles") and then introducing the sensors that enable it. This was rejected as it makes it harder to present a clean, systematic overview of the sensors themselves.

### References

1.  *Introduction to Robotics* course materials, Stanford University.
2.  *Underactuated Robotics* course materials, MIT.
