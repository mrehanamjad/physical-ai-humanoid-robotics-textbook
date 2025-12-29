# Implementation Plan: Chapter 10 – Gazebo Environment Setup

**Feature Branch**: `011-chapter10-gazebo-setup`
**Implementation Plan**: `specs/011-chapter10-gazebo-setup/plan.md`
**Feature Spec**: `specs/011-chapter10-gazebo-setup/spec.md`

## 1. Technical Context & Design Philosophy

This chapter is the first hands-on implementation chapter in Module 2. It transitions learners from the conceptual understanding of digital twins (Chapter 9) to the practical reality of building a simulation environment. The core philosophy is **"success through simplicity."** The goal is to get a robot moving in a simulated world as cleanly and quickly as possible, building the learner's confidence and providing a stable platform for the more complex topics in subsequent chapters (physics, sensors).

The plan prioritizes a single, clear, reproducible path to success. We will make opinionated choices (e.g., recommending a specific Gazebo version) to minimize cognitive overhead for the learner. The focus is on the "what" and "how" of environment setup, while constantly linking back to the "why" established in Chapter 9.

## 2. Constitution Check

- [X] **Specification-Driven Development**: This plan is derived directly from `spec.md`.
- [X] **Technical Correctness**: The plan requires research to ensure all installation and configuration steps are accurate for the recommended software versions.
- [X] **Pedagogical Clarity**: The structure is progressive, starting with installation and ending with a fully communicating simulation.
- [X] **AI-Native Authoring**: AI will be used to draft explanations and generate diagram descriptions.
- [X] **Open Knowledge**: The plan specifies the use of open-source tools (Gazebo, ROS 2).
- [X] **Reproducibility & Traceability**: The testing strategy heavily emphasizes reproducibility on a clean system.

**Gate Evaluation**: Pass. All principles are met.

## 3. Implementation Phases

### Phase 0: Research & Foundation

**Objective**: Make key technical decisions and establish the core architecture for the chapter's examples.

**Tasks**:

1.  **Research Gazebo Versions**:
    *   Task: "Analyze the current state of Gazebo Classic vs. modern Gazebo (Harmonic/Fortress) in the context of ROS 2 Humble/Iron."
    *   Task: "Determine the most stable, well-supported, and feature-complete Gazebo version for humanoid simulation with ROS 2 as of late 2025."
2.  **Architectural Sketch**:
    *   Task: "Draft a description for a diagram showing the Gazebo-ROS 2 integration architecture." The diagram must clearly show:
        *   The Gazebo Server (`gz server`) running the physics and sensor simulation.
        *   The Gazebo Client (`gz client`) as the GUI.
        *   ROS 2 nodes for control and logic.
        *   The `ros_gz_bridge` as the bidirectional communication layer, translating between Gazebo Transport (Ignition) messages and ROS 2 messages.
3.  **Document Key Decisions**:
    *   `research.md` will be created to document the following:
        *   **Gazebo Version**: **Decision: Standardize on Gazebo Harmonic.** Rationale: As of late 2025, Harmonic is the LTS release aligned with ROS 2 Iron and offers the best balance of modern features, stability, and community support compared to the older Gazebo Classic or the bleeding-edge next version. This avoids the fragmentation and confusion of Gazebo Classic's multiple `gazebo-ros-pkgs` vs. modern Gazebo's `ros_gz`.
        *   **Environment Realism**: **Decision: Use simple, low-poly environments.** A ground plane and basic geometric shapes (cubes, spheres) will be used. Rationale: For this introductory chapter, the goal is environment *functionality*, not visual fidelity. Complex visuals would distract from the core learning objectives and introduce unnecessary performance overhead.
        *   **ROS 2 Integration Depth**: **Decision: Focus on the `ros_gz_bridge`.** The plan will cover spawning a URDF via the bridge and using the bridge to pass basic sensor messages (IMU) and receive control commands (`Twist`). Rationale: This is the most fundamental integration point and is a prerequisite for all further work. Custom Gazebo plugins are an advanced topic and will be deferred.
        *   **Setup Order**: **Decision: World-centric first.** The chapter will guide the learner to first launch a world in Gazebo, and *then* spawn the robot into it using a ROS 2 command. Rationale: This separates concerns and makes debugging easier. If the world fails to load, it's a Gazebo issue. If the robot fails to spawn, it's a ROS 2 / URDF issue.
        *   **Theory vs. Practice**: **Decision: 30% conceptual, 70% step-by-step.** This chapter is primarily a "how-to" guide. The conceptual parts will focus on *what* each command does, but the bulk of the text will be clear, numbered, copy-paste-friendly instructions.

**Output**: `specs/011-chapter10-gazebo-setup/research.md`

### Phase 1: Content Generation & Writing Plan

**Objective**: Draft the full hands-on tutorial for the chapter.

**Section-by-Section Writing Plan**:

1.  **Chapter Overview**: Briefly state the goal: "By the end of this chapter, you will have a simulated robot driving around in a virtual world, all controlled by ROS 2."
2.  **Gazebo in the Robotics Ecosystem**: Provide context. Explain that Gazebo is the "physics engine" part of the digital twin concept from Chapter 9.
3.  **Choosing the Right Gazebo Version**: State the decision from `research.md` (use Harmonic) and briefly explain why (LTS, ROS 2 alignment).
4.  **Installing Gazebo and Integration**: Provide `apt install` commands for Gazebo Harmonic and the `ros-iron-ros-gz` packages. Include a verification step.
5.  **Gazebo Workspace and Project Structure**: Define a simple directory structure (`~/gazebo_ws/worlds`, `~/gazebo_ws/models`) and explain its purpose.
6.  **Launching a Basic Gazebo Simulation**:
    *   Show how to run `gazebo` with a simple world file (e.g., `empty.sdf`).
    *   Introduce the `ros_gz_bridge` and explain its role with a simple example (e.g., bridging the `/clock` topic).
7.  **Loading Robots into Gazebo**:
    *   This is the core section. Provide a `ros2 launch` file that does two things:
        1.  Launches the `robot_state_publisher` with the URDF from Chapter 8.
        2.  Uses a `spawn` entity node from `ros_gz_sim` to spawn the robot into the running Gazebo simulation.
8.  **Validating ROS 2 ↔ Gazebo Communication**:
    *   Provide `ros2 topic echo` commands to check for data from a simulated IMU sensor.
    *   Provide a `ros2 topic pub` command to send a `geometry_msgs/msg/Twist` message to move the robot.
9.  **Common Setup Issues**: A troubleshooting FAQ. "Error: `gz` not found" -> Check installation. "Robot falls through world" -> Check URDF collision tags and physics properties. "Topics not bridged" -> Check `ros_gz_bridge` arguments.

**Output**: Draft of the chapter's `.mdx` file.

### Phase 2: Quality Validation & Refinement

**Objective**: Ensure the tutorial is 100% reproducible and easy to follow.

**Quality Validation Checklist**:

- [ ] **Reproducibility**: Has the entire tutorial been run, step-by-step, on a completely clean Ubuntu/ROS 2 installation?
- [ ] **Clarity of Instructions**: Are all commands copy-paste friendly? Are the expected outputs clearly shown?
- [ ] **Usability**: Can a learner with only the prerequisite knowledge complete the chapter without getting stuck?
- [ ] **Diagram Correctness**: Does the Gazebo-ROS 2 architecture diagram accurately represent the message flow through the bridge?
- [ ] **Cross-Chapter Alignment**: Does the URDF used match the one from Chapter 8? Does the explanation of simulation build correctly on Chapter 9? Does it set the stage properly for Chapter 11?

**Testing Strategy**:

1.  **Clean Setup Test**: A separate environment (Docker container or fresh VM) will be used to execute every single command in the chapter from start to finish. Any deviation or error results in a failure.
2.  **Communication Validation**: The final test is to run `ros2 topic echo /imu` and `ros2 topic pub ... /cmd_vel` and verify that the data flows and the robot moves as described.
3.  **Diagram Review**: The generated architecture diagram will be compared against Gazebo's official documentation for accuracy.
4.  **Peer Review**: A team member unfamiliar with the chapter will attempt to follow it. Any point of confusion is a bug that must be fixed.

**Output**: Finalized, tested, and validated chapter content and diagrams.

## 4. Artifacts to be Generated

- `specs/011-chapter10-gazebo-setup/research.md`
- `specs/011-chapter10-gazebo-setup/plan.md` (this file)
- Draft content for the chapter's `.mdx` file.
- Description for the Gazebo-ROS 2 Integration Architecture diagram.

## 5. Next Steps

- Proceed with creating the `research.md` file.
- Begin drafting the chapter tutorial content.
- Handoff to `/sp.tasks` to break down the writing, diagramming, and clean-room testing.