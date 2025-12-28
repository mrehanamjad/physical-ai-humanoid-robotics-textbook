---
title: "Implementation Plan: Chapter 5 – ROS 2 Communication Primitives"
feature: "006-chapter5-ros2-primitives"
status: "draft"
date: "2025-12-27"
---

## 1. Scope and Dependencies

### In Scope
- **Core Communication Patterns**: Detailed conceptual explanations of Topics (publish/subscribe), Services (request/response), and Actions (goal/feedback/result).
- **Node Interaction**: How different nodes communicate using these primitives to achieve complex behaviors.
- **Data Flow**: Tracing the flow of messages, service calls, and action goals/feedback/results between nodes.
- **Decision-Making Criteria**: Guidance on when to choose a Topic, Service, or Action for a given robotic task.
- **Conceptual Executors**: A high-level intuitive explanation of how the ROS 2 runtime processes callbacks and manages concurrent communication.

### Out of Scope
- **Code Implementation**: This chapter remains conceptual. Specific ROS 2 client library API calls (e.g., `create_publisher`, `create_service`) are not covered here but will be in Chapter 6.
- **QoS Settings**: Detailed configuration of Quality of Service profiles for topics.
- **Custom Message Definitions**: How to define and use custom `.msg`, `.srv`, or `.action` files.
- **Distributed Systems Theory**: Deep dives into message queues, fault tolerance, or network protocols beyond the high-level ROS 2 abstraction.

### External Dependencies
- **Chapter 4 (ROS 2 Architecture)**: Assumes a foundational understanding of ROS 2's layered architecture, nodes, and the DDS backbone.
- **Chapter 6 (rclpy Development)**: This chapter provides the conceptual blueprint for the communication patterns that will be implemented using `rclpy`.

## 2. Key Decisions and Rationale

1.  **Order of Introduction (Topics, Services, Actions)**: **Topics → Services → Actions**. This order progresses from the simplest, asynchronous pattern to more complex, goal-oriented patterns.
    - **Rationale**: Topics are the most fundamental and frequently used. Services add request/response. Actions build on services for long-running tasks. This mirrors common pedagogical approaches.

2.  **Conceptual vs. API-Level Detail**: **Predominantly Conceptual**. The chapter will focus on the interaction model and purpose of each primitive.
    - **Rationale**: The spec emphasizes conceptual clarity. API details would distract from understanding the "why" and "how" of interaction types.

3.  **Real-time Intuition (without QoS tuning)**: **Focus on Basic Asynchronicity**. Concepts like "non-blocking" and "callbacks" will be introduced, but not detailed real-time performance tuning.
    - **Rationale**: Providing a feel for concurrency and responsiveness without diving into complex QoS settings is crucial for beginners.

4.  **Use of Humanoid Examples**: **Mixed, with Humanoid Emphasis**. Generic robotics examples (e.g., a "sensor node" and "display node") will be used for simplicity, but always connected back to relevant humanoid contexts.
    - **Rationale**: Generic examples help isolate the communication primitive, but tying it to humanoids reinforces the book's theme.

5.  **Diagram Density vs. Narrative**: **High Diagram Density for Flow**. Each primitive (Topic, Service, Action) will have a dedicated, clear data flow diagram.
    - **Rationale**: Visualizing communication flow is essential for understanding distributed systems.

## 3. Cross-Chapter Interfaces

- **Input**: Builds directly on the `Computational Graph` and `Communication` concepts introduced in Chapter 4.
- **Output**: Provides the communication models (publish/subscribe, request/response, goal/feedback/result) that will be instantiated with `rclpy` in Chapter 6.

## 4. Authoring & Review Workflow

1.  **Research**: Review official ROS 2 communication tutorials and design patterns. Identify common conceptual pitfalls for each primitive.
2.  **Drafting**: Write content section-by-section, ensuring clear distinctions between Topics, Services, and Actions.
3.  **Visuals**: Create dedicated data flow diagrams for Topics, Services, and Actions, showing sender, receiver, and data.
4.  **Review**:
    - Self-review against the validation checklist (Section 7).
    - Peer review for conceptual accuracy and clarity, especially on the distinctions between primitives.
5.  **Finalization**: Refine text and diagrams based on feedback.

## 5. Risk Analysis and Mitigation

- **Risk 1: Confusing Topics vs. Services**: Learners often struggle with when to use which.
  - **Mitigation**: Dedicated section with clear, scenario-based comparison table. Emphasize "continuous data stream" vs. "one-off request."
- **Risk 2: Actions seem Overly Complex**: The goal/feedback/result structure can be intimidating.
  - **Mitigation**: Introduce Actions with a compelling, long-running robotic task (e.g., "walking to a destination") that clearly shows why Topics or Services alone are insufficient. Break down the lifecycle.
- **Risk 3: Abstraction without Grounding**: Explanations become too theoretical.
  - **Mitigation**: Use simple, relatable analogies for each primitive (e.g., newspaper for topics, phone call for services, project manager for actions).

## 6. Evaluation and Validation (Testing Strategy)

- **[ ] Conceptual Accuracy**: Are the explanations for Topics, Services, and Actions fundamentally correct?
- **[ ] Terminology Consistency**: Is `publish/subscribe`, `request/response`, and `goal/feedback/result` used precisely and consistently?
- **[ ] Cross-Chapter Alignment**: Does this chapter flow logically from Chapter 4 and provide a clear foundation for Chapter 6?
- **[ ] Learner Comprehension**: Can a reader correctly choose the appropriate primitive for a given robotic task scenario?
- **[ ] Diagram Clarity**: Do the data flow diagrams accurately represent the communication patterns and enhance understanding?

## 7. Section-by-Section Writing Plan

1.  **Chapter Overview**: State learning objectives. Emphasize ROS 2's role in allowing components to "talk" effectively.
2.  **Recap: The ROS 2 Computational Graph**: Briefly remind learners of nodes from Chapter 4.
3.  **Topics: The Continuous Data Stream (Publish/Subscribe)**:
    - Explain the publish/subscribe model.
    - Use analogy (e.g., newspaper subscriptions).
    - Simple data flow diagram.
    - When to use: continuous sensor data, robot state.
4.  **Services: The Request-Response Interaction**:
    - Explain the request/response model.
    - Use analogy (e.g., phone calls).
    - Simple data flow diagram.
    - When to use: specific queries, discrete commands.
5.  **Actions: For Long-Running Goals with Feedback**:
    - Explain the goal/feedback/result model.
    - Use analogy (e.g., ordering a complex project).
    - Simple data flow diagram showing the full lifecycle.
    - When to use: navigation, manipulation sequences.
6.  **Choosing the Right Tool**: A comparison table (Topics vs. Services vs. Actions) with scenarios.
7.  **Conceptual Executors: Managing the Flow**:
    - Briefly explain that executors are how nodes process incoming messages/requests/goals.
    - Focus on the idea of callbacks and asynchronous processing.
8.  **Summary and Next Steps**: Recap the communication primitives. Highlight that Chapter 6 will teach how to implement these in Python.
9.  **Conceptual Exercises & Reflection**: Scenario-based questions (e.g., "For a robot delivering a package, which communication primitive would you use to report its current location, and which to initiate the delivery?").

## 8. Research Approach

- **Phase 1: Foundation (Upfront Research)**
  - **Task**: Review the official ROS 2 documentation and tutorials specifically on Topics, Services, and Actions to ensure alignment with core concepts.
  - **Task**: Seek out common community explanations and analogies for these primitives.
  - **Output**: A clear and accurate understanding of each primitive and effective pedagogical tools.

- **Phase 2: Analysis (Just-in-Time Research)**
  - Identify clear, simple examples of each primitive in use within robotics contexts.
  - Research common misunderstandings about these primitives to address them proactively.

- **Phase 3: Synthesis**
  - Consolidate information, ensuring clear differentiation and practical application guidance. Track all sources in `research.md`.

## 9. Architectural Decision Records (ADRs) to Document

- `ADR-007: Progressive Introduction of ROS 2 Communication Primitives`
- `ADR-008: Emphasis on Conceptual Models for Communication Primitives`
