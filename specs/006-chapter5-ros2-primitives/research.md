# Research: Chapter 5 – ROS 2 Communication Primitives

## Key Research Findings

### 1. Core Definitions & Analogies

-   **Topics (Publish/Subscribe)**:
    -   **Definition**: A one-to-many, anonymous, asynchronous communication method for continuous data streams. Publishers send messages to a named channel, and any number of subscribers can listen.
    -   **Analogy**: A newspaper or magazine subscription. The publisher prints issues without knowing who reads them, and subscribers receive them without knowing the author directly.
    -   **Canonical Use Case**: Streaming sensor data (camera images, IMU readings), robot state (joint positions, odometry).

-   **Services (Request/Response)**:
    -   **Definition**: A one-to-one, synchronous communication method for remote procedure calls. A client sends a request and waits for a response from a server.
    -   **Analogy**: A phone call or a function call. There is a direct, temporary connection, and the caller waits for an answer.
    -   **Canonical Use Case**: Triggering a discrete action (e.g., "take a picture now"), querying for a piece of data (e.g., "what is the battery voltage?"), changing a configuration setting.

-   **Actions (Goal/Feedback/Result)**:
    -   **Definition**: A one-to-one, asynchronous communication method for long-running, goal-oriented tasks. A client sends a goal, receives a stream of feedback, and gets a final result. The goal can be canceled.
    -   **Analogy**: Assigning a project to a team member or ordering a complex custom product. You give the instructions (goal), get progress updates (feedback), and finally receive the finished product (result).
    -   **Canonical Use Case**: Navigation ("go to the kitchen"), manipulation ("pick up the red ball"), long-running computations.

### 2. Pedagogical Approach

-   **Order of Introduction**: The consensus is to introduce the concepts in order of increasing complexity: **Topics -> Services -> Actions**.
    1.  **Topics** are the simplest and most common, establishing the idea of data streams.
    2.  **Services** introduce the idea of a two-way, blocking interaction.
    3.  **Actions** build on the request/response idea but add asynchronicity, feedback, and cancellation for long tasks.

-   **Conceptual Focus**: For a first introduction, it is critical to focus on the *conceptual model* and the *decision-making criteria* for when to use each primitive. API details should be deferred to a subsequent, practical chapter.

### 3. Common Points of Confusion to Address

-   **Topics vs. Services**: The most common point of confusion. The key distinction is **continuous vs. discrete**. Is it a stream of data or a one-time question? The "newspaper vs. phone call" analogy is effective here.
-   **Services vs. Actions**: When is a task "long-running" enough to justify an Action? The key is the need for **feedback** and/or **cancellation**. If you just need to know when it's done, a Service might suffice. If you need to know *how* it's going or might need to stop it midway, you need an Action.

## References

-   ROS 2 Documentation: [https://docs.ros.org/en/humble/Concepts.html](https://docs.ros.org/en/humble/Concepts.html)
-   ROS 2 Tutorials: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)