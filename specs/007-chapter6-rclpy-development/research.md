# Research: Chapter 6 – Python-Based ROS 2 Development with rclpy

## Key Research Findings

### 1. Beginner-Safe `rclpy` APIs

-   **Nodes**: The `rclpy.node.Node` class is the standard entry point. For beginners, it's best to use a class-based approach where their node inherits from `Node`. This provides a clean, organized structure.
    -   `rclpy.init()` and `rclpy.shutdown()` are essential bookends for any ROS 2 Python application.
    -   `rclpy.spin(node)` is the simplest way to keep a node alive and process callbacks.

-   **Publishers**: `node.create_publisher()` is the core API.
    -   **Key parameters**: `msg_type`, `topic_name`, `qos_profile`. For beginners, using a simple `qos_profile` integer (like 10 for a history depth of 10) is clearest.
    -   `publisher.publish(msg)` is the method for sending a message.

-   **Subscribers**: `node.create_subscription()` is the core API.
    -   **Key parameters**: `msg_type`, `topic_name`, `callback_function`, `qos_profile`.
    -   The concept of the **callback** is the most crucial pedagogical point here. The function passed to `create_subscription` is the entry point for all message processing.

-   **Services**:
    -   **Server**: `node.create_service()` with `srv_type`, `service_name`, and a `callback_function`. The callback must accept `request` and `response` arguments and return the populated `response`.
    -   **Client**: `node.create_client()` with `srv_type` and `service_name`. The client call is asynchronous. Beginners should be taught to use `client.call_async()` and `rclpy.spin_until_future_complete()` to handle the response.

-   **Actions**: The `rclpy.action` module is more complex.
    -   **Server**: `rclpy.action.ActionServer()` with `node`, `action_type`, `action_name`, and an `execute_callback`. The callback receives a `goal_handle` and is expected to return a result. It can also publish feedback via the `goal_handle`.
    -   **Client**: `rclpy.action.ActionClient()`. Sending a goal with `client.send_goal_async()` is non-blocking. Beginners need to be shown how to add callbacks to the returned future to handle acceptance, feedback, and the final result.
    -   **Pedagogical Decision**: Keep the initial Action examples minimal. Focus on the goal/feedback/result flow rather than complex lifecycle management.

### 2. Explaining the `rclpy` Execution Model

-   **The "Spin"**: The most common point of confusion.
    -   **Analogy**: "The Spin is like an event loop or a receptionist." The main thread calls `rclpy.spin(node)`, which effectively hands control over to ROS 2. The spin loop then checks for any incoming events (messages, service calls) and puts the corresponding callback functions into a queue to be executed.
    -   **Visualization**: A diagram showing the main thread "spinning" and an "event queue" being populated with callbacks is highly effective.
-   **Callbacks**: Emphasize that "your code runs inside callbacks." The main part of a ROS 2 node is not a linear script but a collection of event handlers.
-   **Executors**:
    -   For beginners, stick to the `SingleThreadedExecutor` which is the default. Explain it as "processing one callback at a time from the queue."
    -   Briefly mention the `MultiThreadedExecutor` as an advanced option for running callbacks in parallel, but advise against its use until the single-threaded model is fully understood.

### 3. Common `rclpy` Patterns & Performance

-   **Class-Based Nodes**: The dominant and recommended pattern is to encapsulate a node's logic within a class that inherits from `rclpy.node.Node`. This keeps publishers, subscribers, and their logic organized.
-   **Timers**: Use `node.create_timer()` for any periodic task (like publishing at a fixed rate) instead of `time.sleep()`. This integrates properly with the ROS 2 execution model.
-   **Performance**:
    -   **The GIL**: Acknowledge Python's Global Interpreter Lock (GIL). Explain that `MultiThreadedExecutor` provides concurrency, not true parallelism for CPU-bound tasks within a single process.
    -   **Python vs. C++**: Clearly state the tradeoff. Python is excellent for rapid prototyping, UI tools, and high-level logic. For high-frequency, low-latency control loops or heavy computation (like image processing), C++ is the standard choice. This manages learner expectations.

## References

-   ROS 2 `rclpy` API Documentation: [https://docs.ros2.org/latest/api/rclpy/api.html](https://docs.ros2.org/latest/api/rclpy/api.html)
-   ROS 2 Design Articles (Executors, etc.): [https://design.ros2.org/](https://design.ros2.org/)
-   Robotics Stack Exchange for common `rclpy` questions.
