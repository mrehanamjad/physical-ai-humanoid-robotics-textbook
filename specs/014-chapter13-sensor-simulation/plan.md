# Implementation Plan: Chapter 13 – Sensor Simulation

**Feature Branch**: `014-chapter13-sensor-simulation`
**Implementation Plan**: `specs/014-chapter13-sensor-simulation/plan.md`
**Feature Spec**: `specs/014-chapter13-sensor-simulation/spec.md`

## 1. Technical Context & Design Philosophy

This chapter builds directly on the physics simulation concepts from Chapter 11 by answering the question: "How does a robot *perceive* its simulated world?" The philosophy is **"From Perfect to Plausible."** We will first introduce idealized, perfect sensors to establish the basic data flow, and then progressively add layers of realism (noise, latency) to demonstrate why these imperfections are critical for robust AI development.

The plan emphasizes the *data* that sensors produce and how that data gets into the ROS 2 ecosystem. It will focus on the most common sensors in humanoid robotics (cameras, IMUs, LiDAR) and their conceptual underpinnings, rather than providing an exhaustive list of every possible sensor plugin. Practical exercises will involve adding sensor tags to an SDF file and verifying the output on ROS 2 topics.

## 2. Constitution Check

- [X] **Specification-Driven Development**: The plan is directly mapped to the learning objectives and functional requirements of `spec.md`.
- [X] **Technical Correctness**: Research is required to ensure the sensor models and noise parameters are explained correctly and are representative of real-world hardware.
- [X] **Pedagogical Clarity**: The "Perfect to Plausible" approach provides a scaffolded learning experience, building complexity layer by layer.
- [X] **AI-Native Authoring**: AI will be used to draft intuitive explanations for sensor phenomena (e.g., IMU drift) and to generate diagram descriptions.
- [X] **Open Knowledge**: The chapter will focus on standard sensor plugins available in Gazebo and ROS 2.
- [X] **Reproducibility & Traceability**: The testing strategy is built around verifying the data flow from simulation to ROS 2, ensuring the examples are reproducible.

**Gate Evaluation**: Pass. All principles are met.

## 3. Implementation Phases

### Phase 0: Research & Foundation

**Objective**: Define the pedagogical structure for teaching sensor simulation and make key decisions about the level of realism to include.

**Tasks**:

1.  **Research Sensor Models**:
    *   Task: "Analyze the default implementations of Camera, LiDAR, and IMU sensor plugins in Gazebo Harmonic."
    *   Task: "Identify the key configurable parameters for each sensor, especially those related to noise, update rate, and performance."
2.  **Architectural Sketches**:
    *   Task: "Draft a description for a **Simulated Sensor Data Pipeline** diagram." It must illustrate the flow:
        1.  "Gazebo World" (with 3D models).
        2.  Arrow to "Physics Engine" (calculates state).
        3.  Arrow to "Sensor Plugin" (e.g., `gz::sim::sensors::Camera`). The plugin "reads" the state from the physics/rendering engine.
        4.  Arrow to "`ros_gz_bridge`". The plugin publishes a Gazebo Transport message.
        5.  Arrow to a "ROS 2 Topic" (e.g., `/image_raw`). The bridge translates the message.
        6.  Arrow to a "ROS 2 Node" (e.g., a perception algorithm).
    *   Task: "Draft a description for a **Sensor Fidelity Hierarchy** diagram." This will be a pyramid or layered diagram:
        *   **Base (Widest) Layer**: "Level 1: Ideal Sensor". Annotation: "Perfect data, zero noise, zero latency. Good for initial logic testing."
        *   **Middle Layer**: "Level 2: Noisy Sensor". Annotation: "Adds Gaussian noise, bias, and other simple imperfections. Better for robust algorithm development."
        *   **Top (Narrowest) Layer**: "Level 3: Realistic Sensor". Annotation: "Models complex, non-ideal effects like motion blur, rolling shutter, multipath interference. Computationally expensive, used for advanced validation."
3.  **Document Key Decisions**:
    *   `research.md` will be created to document:
        *   **Fidelity vs. Cost**: **Decision: Focus teaching on Level 1 (Ideal) and Level 2 (Noisy).** Rationale: Level 3 realism is an advanced topic and computationally prohibitive for many learners. The most critical pedagogical leap is from perfect to noisy simulation.
        *   **Noise Modeling Depth**: **Decision: Conceptual and parametric.** The chapter will explain *what* Gaussian noise is and *how* to set its mean and standard deviation in an SDF file. It will not derive the noise models themselves.
        *   **Timing and Latency**: **Decision: Introduce as a conceptual challenge.** The chapter will explain *that* sensor delay exists and is a major sim-to-real problem, but will not implement complex latency models. It will focus on setting the sensor's `update_rate`.
        *   **Sensor-Specific vs. Agnostic**: **Decision: A mix.** The chapter will first present sensor-agnostic concepts (the data pipeline, noise, latency). It will then dive into specifics for the three key sensor types (Camera, IMU, LiDAR) as distinct sub-sections.

**Output**: `specs/014-chapter13-sensor-simulation/research.md`

### Phase 1: Content Generation & Writing Plan

**Objective**: Draft the chapter content, building from general principles to specific sensor implementations.

**Section-by-Section Writing Plan**:

1.  **Chapter Overview**: Start by reinforcing the idea that a robot is only as good as its perception, and perception starts with sensors.
2.  **Why Simulate Sensors?**: Briefly recap the safety/cost/scale benefits from Chapter 9, but now in the specific context of training and testing perception algorithms.
3.  **Types of Simulated Sensors**: Categorize sensors as proprioceptive (IMU, encoders) and exteroceptive (cameras, LiDAR), linking back to Chapter 3.
4.  **Camera Simulation**: Explain how a simulated camera is essentially a virtual renderer. Cover `update_rate`, `resolution`, and `hfov` (horizontal field of view). Show the SDF snippet for adding a camera and its `ros_gz_bridge` configuration.
5.  **LiDAR Simulation**: Explain ray casting as the core principle. Cover `update_rate`, `range`, and the number of `samples`/`rays`. Show the SDF snippet and bridge config.
6.  **IMU Simulation**: Explain that an IMU doesn't "see" the world, but instead *samples the state of the physics engine* (acceleration, velocity). This is a critical distinction. Show the SDF snippet.
7.  **Noise, Latency, and Imperfections**: This is the "From Perfect to Plausible" section. Use the Fidelity Hierarchy diagram. Explain what Gaussian noise is intuitively. Show how to add a `<noise>` block to a sensor's SDF configuration.
8.  **Sensor Configuration in Gazebo**: Consolidate the hands-on aspects. Provide a full SDF example of a link with a camera attached. Emphasize the importance of sensor reference frames.
9.  **Using Simulated Sensors**: Show the final step: running `ros2 topic echo` on the bridged topic (e.g., `/image_raw`, `/imu`) to see the data generated by the simulation.

**Output**: Draft of the chapter's `.mdx` file.

### Phase 2: Quality Validation & Refinement

**Objective**: Ensure the sensor data is generated correctly and that the explanations match the observed behavior.

**Quality Validation Checklist**:

- [ ] **Correctness**: Do the simulated sensor plugins generate data in the correct format and coordinate frames?
- [ ] **Realism**: Does adding a noise model to the SDF actually produce noisy data on the ROS 2 topic? Is the magnitude of the noise reasonable?
- [ ] **Learner Intuition**: Is the explanation of how an IMU "samples physics" clear and understandable?
- [ ] **Alignment**: Do the sensor types and concepts align with the real-world sensors described in Chapter 3? Does the SDF configuration build on the knowledge from Chapter 12?
- [ ] **Data Flow Validation**: Does the `ros_gz_bridge` correctly translate Gazebo messages to ROS 2 messages for all example sensors?

**Testing Strategy**:

1.  **Topic Validation**: For each sensor example (IMU, Camera, LiDAR), the primary test is to run the simulation and use `ros2 topic echo` to verify that data is being published on the expected topic with the correct message type.
2.  **Noise Injection Test**:
    *   Run a sensor simulation *without* a noise model and pipe the topic output to a file.
    *   Add a `<noise>` block to the SDF, re-run the simulation, and pipe the output to a second file.
    *   Verify that the two files are different, confirming that the noise model is active.
3.  **Cross-Chapter Consistency Check**: The sensor tags added to the robot's URDF/SDF file must be compatible with the model created in Chapter 12 and the environment from Chapter 10.

**Output**: Finalized, tested, and validated chapter content with working SDF examples.

## 4. Artifacts to be Generated

- `specs/014-chapter13-sensor-simulation/research.md`
- `specs/014-chapter13-sensor-simulation/plan.md` (this file)
- Draft content for the chapter's `.mdx` file.
- Descriptions for two diagrams:
  1.  Simulated Sensor Data Pipeline
  2.  Sensor Fidelity Hierarchy

## 5. Next Steps

- Proceed with creating the `research.md` file.
- Draft the chapter content.
- Handoff to `/sp.tasks` for detailed writing and validation tasks.