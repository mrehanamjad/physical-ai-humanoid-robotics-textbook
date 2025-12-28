# Chapter 4: ROS 2 Architecture and Core Concepts

## Chapter Overview

In the previous chapters, we explored the physical components of a humanoid robot—its sensors and actuators. Now, we turn our attention to the software that brings it all to life. Welcome to the world of the Robot Operating System 2 (ROS 2), the open-source software framework that acts as the central nervous system for a vast number of modern robots.

This chapter is your deep dive into the architecture and core concepts of ROS 2. We will move beyond the simple analogy of a "nervous system" and build a strong mental model of how ROS 2 is structured, why it was designed that way, and how it enables complex robots to function.

Our learning objectives are:
-   To understand the **design philosophy** behind ROS 2 and its evolution from ROS 1.
-   To visualize the **layered architecture** of the ROS 2 stack, from the user-facing code down to the underlying communication system.
-   To define and grasp the four fundamental **communication patterns**: Topics, Services, Actions, and Parameters.
-   To learn how these concepts come together to form a **"computational graph"** that represents a robot's software system.

By the end of this chapter, you will have the conceptual toolkit necessary to understand how ROS 2 systems are designed and to prepare for writing your own robotic applications in the chapters to come.

## Why ROS 2? The Evolution from ROS 1

To understand why ROS 2 is designed the way it is, it helps to look at its predecessor, ROS 1. For over a decade, ROS 1 was the de facto standard for robotics research and education. It was wildly successful because it provided a modular, message-passing system that allowed developers to easily connect disparate software components.

However, ROS 1 was primarily a product of the research lab. As robotics moved from academia to industry, its limitations became apparent:

1.  **Single Point of Failure**: ROS 1 relied on a central process called `roscore` to discover and connect all other nodes. If `roscore` crashed, the entire system would fail.
2.  **Unreliable Networking**: The default networking configuration was not robust and often struggled on lossy wireless networks.
3.  **No Real-Time Support**: ROS 1 was not designed for real-time control, where commands must be executed within strict time deadlines.
4.  **No Security**: It had no built-in security features, meaning any device on the network could send commands to the robot or eavesdrop on its sensor data.

To address these critical shortcomings, ROS 2 was created not as an incremental update, but as a complete redesign. The primary goal was to take the modularity and flexibility of ROS 1 and rebuild it on a foundation that was secure, reliable, and performant enough for industrial and commercial applications.

The single biggest architectural change was the decision to replace ROS 1's custom communication system with the industry-standard **Data Distribution Service (DDS)**. As we will see, this one decision is the key to unlocking most of ROS 2's powerful features, from automatic discovery to real-time control and enhanced security.

## The ROS 2 Design Philosophy

The redesign of ROS 2 was guided by a clear set of principles aimed at addressing the needs of modern robotics. These principles inform every aspect of its architecture.

1.  **Support for Diverse Systems**: Robotics is not one-size-fits-all. A self-driving car has vastly different computational resources than a small drone. ROS 2 is designed to be flexible, supporting systems ranging from tiny microcontrollers with no operating system to large, complex networks of multiple robots.

2.  **Real-Time Control**: Many robotic applications, like controlling a robot arm or maintaining the balance of a walking robot, are **real-time systems**. This means that computations must be completed within a strict deadline. ROS 2 is designed to enable the construction of real-time systems, allowing for fine-grained control over process scheduling and memory management.

3.  **Cross-Platform Compatibility**: While ROS 1 was primarily a Linux-based system, ROS 2 is designed from the ground up to be a true cross-platform framework. It officially supports Linux, Windows, and macOS, making it easier for teams with diverse development environments to collaborate.

4.  **Focus on Production**: ROS 2 is built for "production" environments, not just research labs. This means it prioritizes:
    *   **Reliability**: Systems should be robust to failure.
    *   **Security**: Communication should be authenticated and encrypted.
    *   **Maintainability**: Systems should be easy to debug, deploy, and upgrade.

## The Layered Architecture (The Stack)

To achieve its design goals, ROS 2 is organized into a layered architecture, often called the "ROS 2 stack." This structure is conceptually similar to the OSI model in computer networking, where each layer provides a specific set of services to the layer above it, hiding the complexity of the layers below. This separation of concerns is the key to ROS 2's flexibility and modularity.

![The ROS 2 Layered Architecture](../../static/img/ch04-ros2-stack.png)
*A conceptual diagram of the ROS 2 stack, showing the layers from the hardware up to the user's application.*

Let's explore the stack from the bottom up.

### Layer 1: Hardware
This is the physical robot: the sensors, motors, and compute hardware. ROS 2 is platform-agnostic, meaning it can run on a wide variety of hardware.

### Layer 2: Operating System (OS)
This is the underlying OS that hosts the ROS 2 system. ROS 2 officially supports Linux, Windows, and macOS, and can also run on Real-Time Operating Systems (RTOS) for time-critical applications.

### Layer 3: DDS (Data Distribution Service)
This is the industrial-grade communication middleware that provides the backbone for all of ROS 2's messaging. DDS is a standardized protocol for data-centric, publish-subscribe communication. It handles message serialization, transport, and discovery, allowing ROS 2 nodes to communicate with each other in a reliable and efficient way.

### Layer 4: ROS 2 Middleware Interface (RMW)
DDS is a standard, not a single piece of software. There are many different DDS implementations available, both open-source and commercial. The ROS 2 Middleware Interface (`rmw`) is a crucial abstraction layer that allows ROS 2 to use different DDS implementations interchangeably. This means you can swap out the underlying DDS vendor for one that better suits your project's needs (e.g., for a specific license or performance characteristic) without changing your application code.

### Layer 5: ROS 2 Client Libraries (RCL)
This layer provides the user-facing APIs that most ROS 2 developers will interact with. The ROS Client Libraries (`rcl`) expose the core ROS 2 concepts—nodes, topics, services, etc.—in a consistent way for different programming languages. The two primary client libraries are:
-   `rclcpp`: The C++ client library.
-   `rclpy`: The Python client library.

When you write `rclpy.create_node()`, you are using this layer.

### Layer 6: User Application
This is the top layer, where your application code lives. This layer is composed of one or more ROS 2 **nodes**, each designed to perform a specific task (e.g., a camera driver node, an object detection node, a motor control node). These nodes use the client libraries (like `rclpy`) to communicate with each other and build the overall functionality of the robot.

## DDS: The Industrial-Grade Backbone

As we saw in the architecture stack, ROS 2 is built on top of DDS. This is the single most significant difference between ROS 1 and ROS 2, and it's the source of many of ROS 2's key benefits. But what exactly *is* DDS?

**DDS (Data Distribution Service)** is a middleware standard for data-centric publish-subscribe messaging. It was created for mission-critical industrial and military systems long before ROS 2 existed. Think of applications like air traffic control, smart grids, and financial trading platforms—systems that demand high reliability and real-time performance.

By building on DDS, ROS 2 inherits these powerful capabilities. It's like building a house on a foundation of solid bedrock instead of sand.

### What DDS Provides: The Magic of the Global Data Space

Instead of thinking about sending messages from point A to point B, DDS introduces the concept of a **"Global Data Space."** This is an abstract space where data "lives."

- **Publishers** are applications that *write* or *update* data in this space. For example, a camera node publishes images.
- **Subscribers** are applications that *read* or *are notified about* data in this space. For example, an object detection node subscribes to images.

The key insight is that the publisher doesn't know or care who is subscribing. It just publishes the data. Likewise, the subscriber doesn't know or care who published the data. It just subscribes to the data it needs. DDS handles everything in between:

1.  **Discovery**: When a new node appears on the network, DDS automatically discovers it and its data offerings (publications) and needs (subscriptions). There is no central `roscore` needed. This eliminates the single point of failure from ROS 1.
2.  **Serialization**: It handles the process of converting the data from its in-memory representation into a format that can be sent over the network.
3.  **Transport**: It manages the low-level networking details of actually sending the bytes from one computer to another.
4.  **Reliability**: DDS provides fine-grained control over **Quality of Service (QoS)**. Do you need to make sure every single message arrives (e.g., for a robot's commanded joint positions)? Or is it okay to drop older messages if a newer one is available (e.g., for a video stream)? DDS lets you configure this behavior.

Using an analogy, DDS acts like a combination of a self-organizing phone book and a hyper-efficient postal service. New people (nodes) can join the town, and their address and the mail they send (publications) or receive (subscriptions) are automatically registered. The postal service then figures out the best way to deliver the mail based on the delivery guarantees you've specified. This all happens in the background, allowing your application code to focus on its specific job.

## The Computational Graph: Nodes

While DDS provides the communication backbone, ROS 2 provides the organizational structure. The primary way ROS 2 organizes a robot's software system is through the **computational graph**.

The computational graph is the overall network of all the ROS 2 processes running on the robot. It's a high-level map of the software system. The most fundamental building block of this graph is the **node**.

A **node** is a single, executable unit of computation in the ROS 2 graph. You can think of a node as a small, specialized program that is responsible for one specific task. A well-designed robot software system is composed of many small, single-purpose nodes.

For example, on a humanoid robot, you might have:
-   A `camera_driver` node that gets raw data from the robot's eye cameras.
-   An `object_detector` node that processes the camera images to find objects.
-   A `balance_controller` node that uses data from an IMU sensor to keep the robot upright.
-   A `leg_actuator` node that sends commands to the motors in the robot's legs.

Each of these is a separate node, and together they form the computational graph. By breaking the system down into these small, decoupled nodes, ROS 2 makes the system easier to debug, test, and reuse. If you want to improve your object detection algorithm, you can simply replace the `object_detector` node without having to touch any other part of the system.

## The Computational Graph: Communication

Nodes on their own are just isolated programs. The power of the computational graph comes from how they communicate. ROS 2 provides three primary communication patterns for nodes: Topics, Services, and Actions.

### Topics: The Newspaper Subscription

-   **Analogy**: A newspaper subscription.
-   **Pattern**: One-to-many, asynchronous streaming of data.

**Topics** are the most common communication method in ROS 2. They are used for continuous streams of data, like sensor readings or the robot's state.

A node **publishes** messages to a topic, and any number of other nodes can **subscribe** to that topic to receive the messages. The publisher doesn't know who is subscribing, and the subscribers don't know who is publishing. It's an anonymous, decoupled way to share data.

-   **Example**: A `camera_driver` node continuously publishes images to an `/image_raw` topic. An `object_detector` node subscribes to this topic to get the images, and a `user_interface` node might also subscribe to it to display the video feed to an operator. Both subscribers get the same data stream.

### Services: The Phone Call

-   **Analogy**: A phone call or a function call.
-   **Pattern**: One-to-one, synchronous request-response.

**Services** are used for remote procedure calls. A client node sends a single request message to a server node and waits for a single response message. Unlike topics, services are a two-way, synchronous interaction. The client blocks (waits) until the server completes the request and sends back a response.

-   **Example**: A `robot_control` node might call a `/set_gripper_state` service on a `gripper_actuator` node. The request would contain the desired state (e.g., `open: true`), and the `robot_control` node would wait until the gripper confirms that the action has been completed, returning a response like `success: true`.

### Actions: The Long-Term Project

-   **Analogy**: Assigning a long-term project to a team member.
-   **Pattern**: One-to-one, asynchronous goal with feedback and a final result.

**Actions** are used for long-running, asynchronous tasks that provide feedback while they are executing. They are like services, but with the ability to be canceled and to provide a stream of feedback, not just a single response.

An action client sends a goal to an action server (e.g., "navigate to the kitchen"). The server accepts the goal and starts working on it. While it's working, it can send a stream of feedback messages back to the client (e.g., "distance to goal: 5 meters," "distance to goal: 4 meters"). When the task is finally done, it sends a single result message (e.g., "arrived at destination").

-   **Example**: A `ui_node` sends a goal to a `navigation_server` to "pick up the red ball." The navigation server accepts the goal and starts moving the robot. It sends feedback like the robot's current position. The UI can display this feedback to the user. If the user presses a cancel button, the UI can send a cancel request to the server. When the robot finally grasps the ball, the server sends a result indicating success or failure.

### Parameters: The Configuration Settings

-   **Analogy**: The settings or preferences of an application.
-   **Pattern**: A key-value store for configuring a node.

**Parameters** are used to configure the settings of a node. They allow you to change a node's behavior without having to recompile the code. Parameters can be set at startup or changed dynamically while the node is running. Each node maintains a set of parameters, which can be of types like integers, floats, strings, or booleans.

-   **Example**: Our `object_detector` node might have a parameter called `detection_threshold`. We could set this parameter to a higher value to make the detector less sensitive, or to a lower value to make it more sensitive, all without stopping and restarting the node.


## Putting It Together: A Graph Example

Let's solidify these concepts by sketching out a simple computational graph for a common robotics task: making a robot's head camera point at a detected object.

The system would consist of several nodes communicating over topics and services:

1.  **`/camera_node`**: This node is responsible for the physical camera hardware. It continuously captures images and **publishes** them to the `/image_raw` **topic**.
2.  **`/detector_node`**: This node **subscribes** to the `/image_raw` topic. For each image it receives, it runs an object detection algorithm. If it finds an object, it **publishes** the object's coordinates to the `/detected_object` **topic**.
3.  **`/head_control_node`**: This node's job is to aim the head. It **subscribes** to the `/detected_object` topic. When it receives an object's coordinates, it calculates the necessary pan and tilt angles for the head motors. It then calls the `/set_head_angle` **service** to command the motors.
4.  **`/motor_driver_node`**: This node controls the head's pan and tilt motors. It provides the `/set_head_angle` **service**. When the service is called, it moves the motors to the requested position and returns a response indicating success.

Here is what this looks like as a graph:

![Example Computational Graph](../../static/img/ch04-computation-graph.png)
*A simple computational graph showing how nodes communicate via topics and services to perform a task.*

This example illustrates the power of the ROS 2 graph. Each node has a single, well-defined responsibility. The system is built by composing these simple nodes together into a network. We can easily swap out the `/detector_node` for a different one that uses a better algorithm, and as long as it subscribes to `/image_raw` and publishes to `/detected_object`, the rest of the system doesn't have to change at all. This is the modularity and flexibility that makes ROS 2 so powerful.

## The Execution Model: Processes and Executors

It's crucial to distinguish between the abstract computational graph and what actually happens on the computer. The graph shows the logical connections, but the **execution model** describes how the nodes run.

### Nodes Run in Processes

A **process** is an instance of a program being executed by the operating system. In ROS 2, you can run a single node in its own process, or you can group multiple nodes together to run inside a single process.

-   **Why group nodes?** Grouping nodes into a single process can be much more efficient. If two nodes are in the same process, they can communicate with each other by simply passing pointers to data in memory, without having to serialize and send the data over the network. This is known as **intra-process communication**.

This means there is no one-to-one relationship between a node and a process. The same set of nodes can be deployed in many different ways, depending on the needs of the system.

### Executors: The Schedulers for Callbacks

So if nodes are just collections of callbacks (a function to run when a message arrives on a topic, a function to call when a service is requested), what actually calls these functions? The answer is the **executor**.

An **executor** is the workhorse of a ROS 2 node. It's a library that connects to the underlying DDS network, monitors for all incoming messages and events for the nodes it's managing, and schedules the corresponding callbacks to be executed.

You can think of an executor as a "scheduler for callbacks." It runs a loop that does the following:
1.  Check the network for any new data (a new message on a subscribed topic, a new service request, etc.).
2.  If there is new data, add the corresponding callback function to a queue.
3.  Execute the callbacks in the queue.
4.  Repeat.

A single executor can manage one or more nodes within the same process. By understanding that executors are the link between the network and your code, you gain a much clearer mental model of how a ROS 2 system actually functions.

## Summary and Next Steps

In this chapter, we have built a foundational mental model of the ROS 2 architecture. We've seen how ROS 2 evolved from ROS 1 to meet the demands of production-grade robotics and how its layered architecture provides flexibility and modularity.

Let's recap the key concepts:
-   **Layered Architecture**: ROS 2 is a stack of layers, with the user application at the top and the hardware at the bottom.
-   **DDS**: The industrial-grade middleware that provides the core communication services, including discovery, serialization, and transport.
-   **Computational Graph**: The network of ROS 2 nodes that makes up a robot's software system.
-   **Nodes**: The fundamental, single-purpose building blocks of the graph.
-   **Communication Patterns**:
    -   **Topics**: For one-to-many, asynchronous data streams.
    -   **Services**: For one-to-one, synchronous request-response interactions.
    -   **Actions**: For one-to-one, long-running asynchronous tasks that provide feedback.
-   **Executors**: The workhorses that run within a process, monitoring for network events and scheduling the execution of a node's callbacks.

You now have the complete conceptual vocabulary needed to understand and design ROS 2 systems. In the next chapter, we will finally move from theory to practice. You will learn how to set up your development environment and use the ROS 2 command-line tools to inspect, run, and interact with a live ROS 2 system.

## Conceptual Exercises & Reflection

1.  **Sketch a Graph**: Draw a computational graph for a simple mobile robot that can be driven by a joystick and has a bumper sensor to detect collisions.
    -   What nodes would you create? (e.g., `/joystick_driver`, `/motor_controller`, `/bumper_sensor`).
    -   What topics would they use to communicate?
    -   Would a service or action be appropriate anywhere in this system?

2.  **Choose the Right Tool**: For each of the following scenarios, decide whether a Topic, Service, or Action is the most appropriate communication pattern, and justify your answer.
    -   Sending a continuous stream of IMU sensor data.
    -   Requesting the current battery voltage from a `battery_monitor` node.
    -   Commanding a robot arm to pick up an object from a table, a task that takes 10 seconds.
    -   Triggering a camera to save a single, high-resolution image to disk.

3.  **Layers of Abstraction**: In your own words, explain the purpose of the RMW (ROS 2 Middleware Interface) layer. What key capability does it give to ROS 2 developers?

4.  **Process Placement**: Imagine you have three nodes: a `/camera_node`, a `/image_processing_node`, and a `/motor_control_node`. The camera produces very high-resolution images. To optimize performance, which nodes would you consider placing together in the same process? Why?

5.  **DDS vs. ROS 1**: What are the two most significant advantages that using DDS as a backbone gives ROS 2 compared to ROS 1's old communication system?