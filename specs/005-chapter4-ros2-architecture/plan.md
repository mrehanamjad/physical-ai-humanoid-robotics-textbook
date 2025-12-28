---
title: "Implementation Plan: Chapter 4 – ROS 2 Architecture and Core Concepts"
feature: "005-chapter4-ros2-architecture"
status: "draft"
date: "2025-12-27"
---

## 1. Scope and Dependencies

### In Scope
- **ROS 2 Philosophy**: The "why" behind ROS 2's design, including its evolution from ROS 1, focus on distributed systems, and real-time capabilities.
- **Layered Architecture**: A conceptual breakdown of the ROS 2 stack, from the user-facing client libraries (rclpy, rclcpp) down through the middleware interface (rmw) to the underlying DDS implementation.
- **DDS at a High Level**: An explanation of what the Data Distribution Service (DDS) is and the problems it solves (discovery, reliable communication), without getting lost in its specification details.
- **Computational Graph**: Conceptual definitions of the core ROS 2 entities: Nodes, Topics, Services, and Actions.
- **Execution Model**: A simple explanation of how nodes run within processes and are managed by executors.

### Out of Scope
- **DDS Internals**: No deep dive into DDS wire protocols, Quality of Service (QoS) settings, or specific vendor implementations.
- **Code or CLI Commands**: This chapter is strictly conceptual. No `ros2 run` commands or `rclpy.create_node()` code snippets.
- **ROS 1 Details**: The history of ROS 1 will only be discussed to motivate the design of ROS 2. No details on `roscore` or other ROS 1-specific concepts.
- **Advanced ROS 2 Concepts**: Topics like components, lifecycle nodes, and complex QoS profiles are deferred to later chapters.

### External Dependencies
- **Chapter 2 & 3**: Assumes learners understand the high-level humanoid system architecture and the types of sensor data that need to be communicated.
- **Chapter 5 & 6**: This chapter must provide the solid conceptual foundation required before learners start writing their own ROS 2 nodes and applications.

## 2. Key Decisions and Rationale

1.  **Depth of DDS Internals**: **ROS-Level Abstractions First**. The chapter will focus on what ROS 2 provides. DDS will be treated as the powerful "engine" that enables these features, but readers won't be expected to become DDS experts.
    - **Rationale**: For most ROS 2 users, understanding the ROS API and computation model is sufficient. Deep DDS knowledge is a specialized skill.

2.  **Conceptual vs. Implementation**: **Strictly Conceptual**. The chapter will build a mental model of the system.
    - **Rationale**: The spec mandates a concept-first approach. Mixing in implementation details at this stage would cause cognitive overload and violate the chapter's primary goal.

3.  **Ordering of Topics**: **ROS History → Motivations → Architecture**. The narrative flow will be:
    1.  Briefly, what was ROS 1 and what were its limits?
    2.  Why was ROS 2 created to solve these limits (DDS, real-time, multi-platform)?
    3.  How is the ROS 2 architecture structured to achieve this?
    - **Rationale**: This logical progression tells a story, making the "why" of ROS 2's architecture clear.

4.  **Use of Diagrams vs. Text**: **Diagrams as Anchors**. The chapter will be built around two core diagrams: the layered architecture and a computational graph example. The text will serve to explain these diagrams in detail.
    - **Rationale**: Abstract software architectures are best understood visually.

5.  **Completeness vs. Cognitive Load**: **Minimize Cognitive Load**. The chapter will deliberately omit many advanced or secondary ROS 2 features to focus on the absolute core concepts.
    - **Rationale**: The goal is to build a strong, accurate initial mental model. Completeness can be achieved progressively in later chapters.

## 3. Cross-Chapter Interfaces

- **Input**: Takes the abstract system blocks from Chapter 2 (e.g., Perception, Control) and the data types from Chapter 3 (e.g., images, point clouds) as things that need to be connected.
- **Output**: Provides the terminology and architectural model (nodes, topics, etc.) that will be implemented directly in Chapters 5 and 6.

## 4. Authoring & Review Workflow

1.  **Research**: Study the official ROS 2 design articles and core documentation to ensure the conceptual explanation is accurate and aligned with the developers' intent.
2.  **Drafting**: Write the chapter following the narrative flow decided above.
3.  **Visuals**: Create the layered architecture and computational graph diagrams. These must be extremely clear and well-annotated.
4.  **Review**:
    - Self-review against the validation checklist (Section 7).
    - Peer review by someone familiar with ROS 2 to check for conceptual errors or misleading simplifications.
5.  **Finalization**: Refine text and diagrams based on feedback.

## 5. Risk Analysis and Mitigation

- **Risk 1: Abstract Overload**: The layers of abstraction (rcl, rmw, DDS) can be confusing.
  - **Mitigation**: Use a strong, consistent analogy throughout the chapter (e.g., comparing it to the OSI model in computer networking). Every layer's purpose must be clearly defined.
- **Risk 2: Inaccurate Simplification**: Simplifying a concept (like an executor) to the point that the mental model is flawed.
  - **Mitigation**: The research phase must identify the "essential truth" of each concept. The explanation must be simple, but not wrong. For example, an executor is a "scheduler for callbacks," not just a "thing that runs the node."
- **Risk 3: Confusing the Graph with Reality**: Learners might confuse the abstract computational graph with the physical layout of the robot.
  - **Mitigation**: Explicitly state that multiple nodes can run in a single process on one computer, and that nodes communicate over the network, regardless of where they physically run.

## 6. Evaluation and Validation (Testing Strategy)

- **[ ] Architectural Consistency**: Does the explanation of the layered stack make sense? Is the flow from user code to DDS clear?
- **[ ] Terminology Validation**: Are `node`, `executor`, `middleware`, and `DDS` defined clearly and used consistently?
- **[ ] Prior Chapter Alignment**: Does the chapter naturally follow from the concepts in Chapters 2 and 3?
- **[ ] Forward-Compatibility**: Does this chapter provide the necessary and sufficient conceptual background for learners to succeed in Chapters 5, 6, and 7?
- **[ ] Diagram Accuracy**: Are the diagrams clear, correct, and well-integrated with the text?

## 7. Section-by-Section Writing Plan

1.  **Chapter Overview**: State learning objectives. Introduce ROS 2 as the "nervous system" of the robot.
2.  **Why ROS 2? The Evolution from ROS 1**: Briefly explain ROS 1's limitations (e.g., single point of failure) to motivate ROS 2's creation.
3.  **The ROS 2 Design Philosophy**: Discuss the goals: distributed, real-time, modular.
4.  **The Layered Architecture (The Stack)**: Introduce the main diagram. Explain each layer's role (Hardware -> OS -> DDS -> ROS 2 Middleware -> ROS 2 Client Libraries -> User Code).
5.  **DDS: The Industrial-Grade Backbone**: Explain what DDS is (a standard for pub/sub) and what it provides (discovery, reliability). Use an analogy like a "magic, self-organizing phone book and delivery service."
6.  **The Computational Graph: Nodes**: Define a node as a single-purpose, executable unit of computation.
7.  **The Computational Graph: Communication**: Define Topics, Services, and Actions conceptually using the "newspaper vs. phone call vs. project" analogy.
8.  **Putting It Together: A Graph Example**: Use the second main diagram to show how a sensor driver node might publish images to a topic, and an object detection node subscribes to it.
9.  **The Execution Model: Processes and Executors**: Explain that nodes are run inside OS processes, and executors are what make the callbacks happen.
10. **Summary and Next Steps**: Recap the core concepts and state that the next chapter will put this theory into practice.
11. **Conceptual Exercises & Reflection**: Write questions aligned with acceptance criteria (e.g., "Sketch a computational graph for a robot that follows a person.").

## 8. Research Approach

- **Phase 1: Foundation (Upfront Research)**
  - **Task**: Thoroughly read the official ROS 2 documentation, particularly the "Concepts" and "Design" sections.
  - **Task**: Find and review high-quality presentations or articles that explain the ROS 2 architecture to beginners.
  - **Output**: A solid understanding of the core developers' intent and a set of reference explanations.

- **Phase 2: Analysis (Just-in-Time Research)**
  - When writing each section, find strong analogies to explain abstract concepts (e.g., DDS, executors).
  - Research the most common points of confusion for ROS 2 beginners to proactively address them.

- **Phase 3: Synthesis**
  - Consolidate all information into a clear, logical narrative, ensuring it aligns with the official ROS 2 documentation. Track all sources in `research.md`.

## 9. Architectural Decision Records (ADRs) to Document

- `ADR-005: Conceptual-First Introduction to ROS 2 Architecture`
- `ADR-006: Abstraction of DDS in Foundational ROS 2 Chapters`
