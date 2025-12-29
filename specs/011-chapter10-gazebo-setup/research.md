# Research: Chapter 10 – Gazebo Environment Setup

## 1. Key Decisions & Rationales

### Decision 1: Standardize on Gazebo Harmonic + ROS 2 Iron

-   **Decision**: This chapter will standardize all instructions and examples on Gazebo Harmonic, the Long-Term Support (LTS) release that is officially paired with ROS 2 Iron.
-   **Rationale**: The robotics simulation landscape has been fragmented. Gazebo Classic (v11) is the old standard but is now in maintenance. Modern Gazebo (formerly Ignition) has multiple named releases (Fortress, Garden, Harmonic). For learners, this is a major source of confusion. By selecting a single, modern, LTS version that is officially supported by the target ROS 2 distribution (Iron), we provide the clearest and most stable path to success. This avoids legacy packages (`gazebo_ros_pkgs`) in favor of the modern `ros_gz` bridge, which is the current and future standard.
-   **Alternatives Considered**:
    -   *Gazebo Classic (v11)*: Rejected because it is a legacy technology and would require teaching an older, more confusing set of ROS integration packages.
    -   *Gazebo Garden*: Rejected because it is not an LTS release, making it less stable for a foundational textbook.
    -   *Multiple Versions*: Rejected because it would introduce branching logic and immense cognitive overhead for a learner who is just starting with simulation.

### Decision 2: Prioritize Low-Poly, Functional Environments

-   **Decision**: All example environments in this chapter will be simple and functional, using basic geometric shapes (e.g., a ground plane, walls made of boxes) rather than photorealistic models.
-   **Rationale**: The primary learning objective is to understand the setup and communication pipeline. High-fidelity visual environments would be a distraction, increase simulation load times, and raise the hardware requirements for learners. Functionality is prioritized over aesthetics for this foundational chapter.
-   **Alternatives Considered**:
    -   *Photorealistic Worlds*: Rejected as overkill for an introductory setup chapter. This topic is better suited for later chapters on sensor simulation or visualization with tools like Isaac Sim.

### Decision 3: Focus on `ros_gz_bridge` for Integration

-   **Decision**: The chapter will exclusively use the `ros_gz` suite of tools, specifically the `ros_gz_bridge`, to connect ROS 2 with Gazebo. Custom Gazebo plugins will not be covered.
-   **Rationale**: The `ros_gz_bridge` is the standard, modern tool for this purpose. It is declarative and easier for beginners to understand than writing a custom C++ Gazebo plugin. Mastering the bridge is the most critical and versatile integration skill.
-   **Alternatives Considered**:
    -   *Custom Plugins*: Rejected as an advanced topic. It requires C++ knowledge and understanding of Gazebo's internal API, which is beyond the scope of this introductory chapter.

### Decision 4: Adopt a World-Centric Setup Workflow

-   **Decision**: The tutorial will instruct learners to first launch the Gazebo world and server, and *then*, in a separate step, use a ROS 2 command to spawn the robot model into the running simulation.
-   **Rationale**: This decouples the environment from the robot, which is a powerful debugging methodology. If the world doesn't load, the problem is with the Gazebo installation or the world file. If the robot doesn't appear, the problem is with the URDF, the ROS 2 launch file, or the spawn command. This separation prevents learners from facing a single, monolithic failure state.
-   **Alternatives Considered**:
    -   *Monolithic Launch*: A single launch file that starts Gazebo and spawns the robot. Rejected because it's harder to debug; a failure in one part can cascade and produce confusing error messages.

## 2. Architectural Sketch Description

**Diagram Title**: Gazebo-ROS 2 Integration Architecture

**Description for Generation**:
Create a system diagram with two main columns: "ROS 2 Domain" on the left and "Gazebo Transport Domain" on the right. In the middle, a central component labeled "`ros_gz_bridge`" connects them.

1.  **Left Column: "ROS 2 Domain"**
    *   Contains several boxes representing ROS 2 nodes.
    *   A box at the top labeled "**Your Control Node**" has an arrow pointing out labeled with a ROS 2 message type, e.g., `geometry_msgs/msg/Twist`. This arrow goes to the bridge.
    *   A box at the bottom labeled "**Your Perception Node**" has an arrow pointing *into* it from the bridge, labeled with a ROS 2 message type, e.g., `sensor_msgs/msg/Imu`.
    *   Another box labeled "`robot_state_publisher`" is present.

2.  **Right Column: "Gazebo Transport Domain"**
    *   A large box labeled "**Gazebo Sim Server**" encloses smaller components:
        *   "Physics Engine"
        *   "Sensor Models"
        *   "Robot Model (SDF)"
    *   An arrow points from the bridge *into* the "Robot Model (SDF)" component, labeled with a Gazebo message type, e.g., `gz.msgs.Twist`.
    *   An arrow points from the "Sensor Models" component *out* to the bridge, labeled with a Gazebo message type, e.g., `gz.msgs.IMU`.

3.  **Center Component: `ros_gz_bridge`**
    *   This box sits between the two columns.
    *   The arrows from both sides connect to it, showing its role as a translator.
    *   Add a small text annotation: "Translates messages between ROS 2 and Gazebo Transport protocols."

This diagram visually explains that ROS 2 and Gazebo are separate programs that communicate through a specialized translation layer (the bridge).
