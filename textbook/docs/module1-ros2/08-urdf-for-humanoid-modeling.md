
# Chapter 8: URDF for Humanoid Robot Modeling


## Chapter Overview

Welcome to Chapter 8. In this chapter, we will learn how to define the physical structure of a humanoid robot using the Unified Robot Description Format (URDF). URDF is an XML-based file format that allows us to model the kinematic and dynamic properties of a robot. We will cover the fundamental building blocks of a URDF file, including:

- **Links**: The rigid parts of the robot.
- **Joints**: The connections between links that allow for motion.
- **Kinematic Trees**: The hierarchical structure of links and joints that defines the robot's skeleton.

By the end of this chapter, you will be able to create a URDF model for a simple humanoid robot and visualize it in RViz2. This "body schema" is the foundation for all a robot's physical interactions with the world, from simple movement to complex manipulation and simulation.

## What is URDF?

The Unified Robot Description Format (URDF) is a standard format for representing a robot model in ROS 2. It uses XML to describe the robot's physical properties, including:

- **Links**: The rigid bodies of the robot.
- **Joints**: The connections between links, defining how they can move relative to each other.
- **Visuals**: The 3D models used to visualize the robot's links.
- **Collisions**: The simplified shapes used by the physics engine to detect collisions.
- **Inertial Properties**: The mass and rotational inertia of each link, crucial for dynamic simulation.

A URDF file describes the robot as a tree of links and joints, starting from a single root link. This tree structure is known as the robot's **kinematic tree**. ROS 2 uses this model to perform kinematic and dynamic calculations, as well as for visualization and simulation.

### What URDF Can and Cannot Do

URDF is a powerful tool for describing the kinematics and dynamics of a robot, but it has some limitations:

**What URDF is good for:**
- Describing the kinematic tree of a robot.
- Specifying the visual and collision properties of links.
- Defining the inertial properties of links.
- Modeling serial chains of robots.

**What URDF is not good for:**
- Modeling closed kinematic chains (loops). For example, a four-bar linkage cannot be directly modeled in URDF.
- Describing the robot's environment.
- Specifying the robot's control logic.
- Including sensor models in a standard way (although it can be extended for specific simulators like Gazebo).

For more complex scenarios, you may need to use other formats, such as the **Simulation Description Format (SDF)**, which is used by Gazebo. However, for many robotics applications, URDF is a perfectly suitable and widely used format.

## The `<robot>` Tag

Every URDF file has a root element called `<robot>`. This tag contains the entire robot description and is given a name. All other elements, such as `<link>` and `<joint>`, are nested within this tag.

Here is a minimal example of a URDF file:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links and joints go here -->
</robot>
```

The `name` attribute of the `<robot>` tag is important for identifying the robot model. Inside this tag, we will define all the links and joints that make up our humanoid robot.

## Defining Links: Visual, Collision, and Inertial Properties

For our first link, we will create a simple cylindrical torso. A link is a rigid body of the robot. Each link has a name and can have three optional properties defined as sub-elements: `<visual>`, `<collision>`, and `<inertial>`.



```xml

<link name="my_link">

  <visual>

    <origin xyz="0 0 0" rpy="0 0 0" />

    <geometry>

      <box size="1 1 1" />

    </geometry>

    <material name="white">

      <color rgba="1 1 1 1" />

    </material>

  </visual>

  <collision>

    <origin xyz="0 0 0" rpy="0 0 0" />

    <geometry>

      <box size="1 1 1" />

    </geometry>

  </collision>

  <inertial>

    <origin xyz="0 0 0" rpy="0 0 0" />

    <mass value="1" />

    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />

  </inertial>

</link>

```



- **`<visual>`**: This element describes how the link should look in visualizations like RViz2.

    - `<origin>`: The pose of the visual geometry relative to the link's origin.

    - `<geometry>`: The shape of the visual. This can be a simple shape (`<box>`, `<cylinder>`, `<sphere>`) or a 3D mesh file (`<mesh filename="package://my_package/meshes/my_mesh.stl" />`).

    - `<material>`: The color and texture of the visual.



- **`<collision>`**: This element defines the collision geometry of the link. It is used by the physics engine to calculate collisions. To improve performance, the collision geometry is often a simplified version of the visual geometry.

    - `<origin>`: The pose of the collision geometry relative to the link's origin.

    - `<geometry>`: The shape of the collision geometry.



- **`<inertial>`**: This element specifies the inertial properties of the link, including its mass, center of mass, and inertia matrix. These properties are crucial for accurate dynamic simulation.

    - `<origin>`: The pose of the center of mass relative to the link's origin.

    - `<mass>`: The mass of the link in kilograms.

    - `<inertia>`: The 3x3 rotational inertia matrix.



## Defining Joints: Types, Axes, and Limits

A **joint** connects two links and defines their relative motion. Every joint has a parent link and a child link. The `type` attribute of the `<joint>` tag specifies the type of motion allowed. Common joint types include:

- **`revolute`**: A hinge joint that rotates around a single axis. It has defined upper and lower limits.
- **`continuous`**: A joint that can rotate continuously around an axis (like a wheel).
- **`prismatic`**: A sliding joint that moves along an axis. It also has upper and lower limits.
- **`fixed`**: A rigid connection between two links with no motion. This is very useful for rigidly connecting parts of the robot that are modeled as separate links, for example, a sensor to a link.
- **`floating`**: Allows motion in all 6 degrees of freedom.
- **`planar`**: Allows motion in a plane.

The `<joint>` tag also contains the following important elements:

- **`<parent link="..."/>` and `<child link="..."/>`**: These tags specify the names of the two links being connected.
- **`<origin xyz="..." rpy="..."/>`**: This tag defines the transform (position and orientation) of the joint's frame relative to the parent link's frame. The `xyz` attribute specifies the translational offset, and the `rpy` attribute specifies the rotational offset in roll, pitch, and yaw.
- **`<axis xyz="..."/>`**: For `revolute` and `prismatic` joints, this tag specifies the axis of motion in the joint's coordinate frame.
- **`<limit lower="..." upper="..." effort="..." velocity="..."/>`**: For `revolute` and `prismatic` joints, this tag sets the limits of motion.
    - `lower` and `upper`: The lower and upper joint limits (in radians for revolute and meters for prismatic).
    - `effort`: The maximum torque (for revolute) or force (for prismatic) the joint can exert.
    - `velocity`: The maximum speed of the joint.
- **`<dynamics damping="..." friction="..."/>`**: This tag specifies the physical properties of the joint, such as damping and friction. These are important for realistic simulation.
- **`<safety_controller soft_lower_limit="..." soft_upper_limit="..." k_position="..." k_velocity="..."/>`**: This tag is used to specify a safety controller that will prevent the joint from exceeding its limits.

## Building a Hierarchical Skeleton: The Kinematic Tree

By connecting links with joints, we build a **kinematic tree**. This is a hierarchical structure with a single root link. The root link is the only link in the tree that does not have a parent joint. All other links have exactly one parent.

For a humanoid robot, the `base_link` is often chosen as the root link. This link is typically located at the center of the robot's torso. From the `base_link`, we can define kinematic chains for the arms, legs, and head.

For example, a simple arm might consist of the following chain:

`torso` -> `shoulder_joint` -> `upper_arm` -> `elbow_joint` -> `forearm`

Here, `torso` is the parent of `shoulder_joint`, `upper_arm` is the child of `shoulder_joint` and the parent of `elbow_joint`, and so on. This chain defines the structure and motion of the arm.

Here is a diagram illustrating this link-joint hierarchy:

![Kinematic Tree](/img/ch08-kinematic-tree.png)

And here is a diagram showing the relationship between the coordinate frames of the links:

![Coordinate Frames](/img/ch08-frames.png)


## A Simple Example: A Two-Link Arm

Let's put everything we've learned together to create a simple two-link arm. This arm will have a base, an upper arm, and a forearm.

Here is the complete URDF file, which you can find in `textbook/urdf/chapter8/simple_arm.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="forearm">
    <visual>
      <geometry>
        <cylinder length="0.8" radius="0.08"/>
      </geometry>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

In this example, we have three links (`base_link`, `upper_arm`, `forearm`) and two revolute joints (`shoulder_joint`, `elbow_joint`). The `base_link` is the root of our kinematic tree. The `shoulder_joint` connects the `base_link` to the `upper_arm`, and the `elbow_joint` connects the `upper_arm` to the `forearm`.

## Adding Sensors to the Model

URDF can be extended to include sensor models. This is particularly useful when working with simulators like Gazebo. While the standard URDF specification does not have a `<sensor>` tag, we can add it for use with specific tools.

A common practice is to create a new link for the sensor and attach it to an existing link with a `fixed` joint. For example, to add a camera to the forearm of our simple arm, we could do the following:

1.  Create a new link called `camera_link`.
2.  Define a `fixed` joint called `camera_joint` that connects `forearm` (the parent) to `camera_link` (the child).
3.  The `<origin>` of this joint would specify the camera's position and orientation relative to the forearm.

While we won't go into the details of defining sensor properties in this chapter, it is important to know that the kinematic structure of your robot model can be extended to include sensors.

## Modular Design with Xacro

As robot models become more complex, URDF files can become very large and difficult to maintain. **Xacro** (XML Macros) is a tool that helps to make URDF files more readable and modular. Xacro allows you to define reusable macros, use mathematical expressions, and include other files.

### Xacro Properties

Properties are like constants in a programming language. They allow you to define a value once and reuse it throughout the file. This is especially useful for defining dimensions, masses, and other parameters that might change.

```xml
<xacro:property name="M_PI" value="3.1415926535897931" />
<xacro:property name="arm_length" value="0.5" />
```

You can then use these properties in your URDF:

```xml
<origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
```

### Xacro Macros

Macros are like functions. They allow you to define a block of XML that can be reused multiple times with different parameters. This is extremely useful for creating repetitive structures like arms, legs, or fingers.

```xml
<xacro:macro name="default_inertial" params="mass">
  <inertial>
    <mass value="${mass}" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
</xacro:macro>
```

You can then instantiate this macro inside a link:

```xml
<xacro:default_inertial mass="10"/>
```

### Including Other Files

Xacro also allows you to include other Xacro files. This is useful for organizing a complex robot model into smaller, more manageable files.

```xml
<xacro:include filename="$(find my_robot_description)/urdf/materials.xacro" />
```

Here is our simple arm example rewritten using Xacro. You can find this file in `textbook/urdf/chapter8/simple_arm.xacro`:

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="cylinder_link" params="name length radius mass">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0}"/>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <inertial>
        <mass value="${mass}" />
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
      </inertial>
    </link>
  </xacro:macro>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <xacro:cylinder_link name="upper_arm" length="1" radius="0.1" mass="1"/>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-${M_PI/2}" upper="${M_PI/2}" effort="10" velocity="1"/>
  </joint>

  <xacro:cylinder_link name="forearm" length="0.8" radius="0.08" mass="0.8"/>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-${M_PI/2}" upper="${M_PI/2}" effort="10" velocity="1"/>
  </joint>

</robot>
```

In this Xacro file, we have defined a macro called `cylinder_link` to create a cylindrical link. We then use this macro to create the `upper_arm` and `forearm` links. We also use a property to define the value of PI, which is then used in the joint limits.

To convert a Xacro file to a URDF file, you can use the following command:

```bash
xacro textbook/urdf/chapter8/simple_arm.xacro > textbook/urdf/chapter8/simple_arm.urdf
```

## Visualizing the Model with RViz2

Once you have a URDF file, you can visualize it in RViz2, the 3D visualization tool for ROS 2. To do this, you will need to launch the `robot_state_publisher` and `joint_state_publisher` nodes.

### The `robot_state_publisher` Node

The `robot_state_publisher` node is responsible for publishing the state of the robot to the `/tf` topic. It reads the URDF from the `robot_description` parameter and the joint states from the `/joint_states` topic, and then calculates the 3D poses of the links and publishes them as TF transforms.

### The `joint_state_publisher` Node

The `joint_state_publisher` node publishes the `/joint_states` topic. It provides a simple GUI with sliders that you can use to change the joint angles. This is useful for testing and debugging your robot model. For more complex robots, you would typically have a controller that publishes the joint states.

### Launch File

Here is a simple launch file to visualize our simple arm. To use this, you need to have a ROS 2 package and place this launch file in the `launch` directory.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    urdf_file_name = 'simple_arm.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('your_package_name'),
        'urdf',
        'chapter8',
        urdf_file_name)

    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('your_package_name'), 'rviz', 'urdf.rviz')])
    ])
```

You will need to replace `your_package_name` with the name of your ROS 2 package. You will also need to create an RViz2 configuration file (`urdf.rviz`) to display the robot model.

### RViz2 Configuration

In RViz2, you will need to add a `RobotModel` display and set the `robot_description` topic to `/robot_description`. You may also want to add a `TF` display to visualize the coordinate frames. You can save your RViz2 configuration to a file and load it automatically using the launch file.


## Validating the URDF Model

Before using a URDF file, it is a good practice to check it for syntax errors. ROS 2 provides a tool called `check_urdf` for this purpose.

To use `check_urdf`, you can run the following command:

```bash
check_urdf textbook/urdf/chapter8/simple_arm.urdf
```

If the file is valid, the tool will print a description of the robot's structure. If there are any errors, it will print an error message indicating the location of the error in the file.

## A Complete Humanoid Model (Simplified)

Now, let's create a slightly more complex model of a humanoid torso with a head. This will demonstrate how to build up a kinematic chain.

Here is the URDF file, which you can find in `textbook/urdf/chapter8/torso_with_head.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

In this model, we have a `base_link` representing the torso and a `head` link. The `neck_joint` is a revolute joint that allows the head to rotate around the z-axis. This is a simple example, but it shows the basic principles of building a humanoid robot model. From here, you could add arms, legs, and sensors to create a complete humanoid robot.

## Beyond URDF: SDF and other formats

While URDF is the most common format for describing robots in ROS, it is not the only one. The **Simulation Description Format (SDF)** is another popular format, especially in the context of the Gazebo simulator.

SDF is a more expressive format than URDF and can be used to describe not only the robot, but also the entire simulation environment, including lighting, physics, and other objects. SDF can also model closed kinematic chains, which is a limitation of URDF.

For advanced simulation, you may find that you need to use SDF. However, it is often possible to convert a URDF model to an SDF model. There are also tools that allow you to use URDF models directly in Gazebo, with some limitations.

Another format worth mentioning is the **Kinematics and Dynamics Library (KDL)**. KDL is a C++ library that provides a data structure for representing kinematic chains and a set of solvers for performing kinematic and dynamic calculations. The `robot_state_publisher` node uses KDL internally to perform its calculations.

For the purposes of this textbook, we will focus on URDF, as it is the most fundamental and widely used format in ROS.

## Common Issues and Debugging

When working with URDF files, you may encounter some common issues. Here are a few tips for debugging your URDF models:

- **Check your syntax**: URDF files are XML files, so they must have correct XML syntax. Make sure all your tags are properly nested and closed.
- **Use `check_urdf`**: As mentioned earlier, the `check_urdf` tool is a great way to catch syntax errors and other issues in your URDF file.
- **Visualize in RViz2**: RViz2 is your best friend when it comes to debugging URDF files. You can visually inspect your robot model and check that all the links and joints are in the correct place.
- **Check your frames**: Use the TF display in RViz2 to visualize the coordinate frames of your links. This can help you to debug issues with joint origins and axes.
- **Isolate the problem**: If you are working on a complex model, try to isolate the problem by creating a simpler model with just a few links and joints.

By following these tips, you can save yourself a lot of time and frustration when working with URDF files.

## Hands-On Exercises



Now it's your turn to practice what you've learned.



1.  **Build a Simple Leg**: Create a new URDF file called `simple_leg.urdf`.

    -   The leg should have a `hip_joint` (revolute), a `thigh` link, a `knee_joint` (revolute), and a `shin` link.

    -   The `thigh` and `shin` links should be cylinders.

    -   The `hip_joint` should rotate around the X-axis, and the `knee_joint` should rotate around the Y-axis.

    -   Add reasonable limits to the joints.



2.  **Extend the Humanoid**: Create a `humanoid.xacro` file and use it to combine the `torso_with_head.urdf` with your `simple_leg.urdf`.

    -   Create a macro for the leg in a separate `leg.xacro` file.

    -   Include the `leg.xacro` file in your `humanoid.xacro` file.

    -   Instantiate the leg macro twice, once for the left leg and once for the right leg. You will need to use different origins for the two legs.



3.  **Visualize Your Creation**: Create a launch file to visualize your new humanoid model in RViz2.

    -   Your launch file should start the `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2` nodes.

    -   You will need to create a new RViz2 configuration file to display your humanoid model.



These exercises will give you a chance to apply the concepts you've learned in this chapter and build a more complete robot model. Good luck!
