# Research for Chapter 7: Launch Files, Parameters, and System Orchestration

## T003: Python Launch Files vs. `ros2 run` CLI

*   **`ros2 run`**:
    *   **Purpose**: For launching a single, executable node from a package.
    *   **Usage**: `ros2 run <package_name> <executable_name>`
    *   **Strengths**: Simple, quick for testing individual nodes, good for debugging.
    *   **Limitations**: Can only launch one node at a time, limited configuration options (only via command-line arguments).

*   **Python Launch Files (`ros2 launch`)**:
    *   **Purpose**: For describing and launching a system of multiple nodes and their configurations.
    *   **Usage**: `ros2 launch <package_name> <launch_file_name.py>`
    *   **Strengths**:
        *   **Multi-node Management**: Can launch and manage dozens of nodes from a single command.
        *   **Configuration**: Allows for setting parameters, remapping topics, setting environment variables, and more for each node.
        - **Automation**: Automates the entire startup process, ensuring nodes are started in the correct order and with the right settings.
        *   **Flexibility**: As they are Python scripts, they can include logic, such as conditionals and loops, for more advanced configurations.
    *   **Decision**: For this chapter, we will focus on Python launch files as they are the standard way to run any non-trivial ROS 2 system. We will introduce `ros2 run` as a basic tool for single-node testing.

## T004: ROS 2 Parameter System for Beginners

*   **Concept**: Parameters are configurable settings for a node. They are key-value pairs (e.g., `use_sim_time: true`).
*   **Declaration**:
    *   Nodes must explicitly declare the parameters they accept. This prevents typos and makes the node's interface clear.
    *   In Python, this is done with `self.declare_parameter('param_name', 'default_value')`.
*   **Setting Parameters**:
    *   **Launch File**: The most common way. Parameters can be passed to a `Node` action in the launch file.
    *   **YAML File**: For larger sets of parameters, they can be defined in a `.yaml` file and loaded by the launch file. This is the best practice for organization.
    *   **Command Line**: For quick tests, parameters can be overridden from the command line using `ros2 run <package_name> <executable_name> --ros-args -p <param_name>:=<value>`.
*   **Getting Parameters**:
    *   Inside a node, a parameter's value can be retrieved with `self.get_parameter('param_name').get_parameter_value()`.
*   **Scope for this Chapter**:
    *   Focus on declaring parameters with default values.
    *   Show how to pass parameters from a launch file.
    *   Introduce the use of YAML files for organizing parameters, as this is a crucial skill for real-world projects.

## T005: Lifecycle Nodes

*   **Concept**: Lifecycle nodes (or "managed nodes") are nodes with a standardized state machine. This allows for more controlled startup, shutdown, and error handling of a system.
*   **States**: The primary states are `unconfigured`, `inactive`, `active`, and `finalized`. Transitions between these states are triggered by a supervisor process.
*   **Use Case**: Essential for complex systems where the order of initialization matters. For example, a driver node for a camera should be fully configured and ready before the image processing node starts trying to use it.
*   **Scope for this Chapter**:
    *   A full implementation of lifecycle nodes can be complex for beginners.
    *   The best approach for this introductory chapter is to provide a **conceptual overview**.
    *   We should explain *what* they are, *why* they are useful (the problem they solve), and show the state transition diagram.
    - We can mention that they are an advanced topic and will be covered in more detail in a later section or an appendix if needed. This keeps the core content focused on the fundamentals of launching and parameterization.
