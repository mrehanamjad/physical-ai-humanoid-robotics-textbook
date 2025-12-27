# Feature Specification: Chapter 5 – ROS 2 Communication Primitives

**Feature Branch**: `006-chapter5-ros2-primitives`  
**Created**: 2025-12-27
**Status**: Draft  
**Input**: User description: "Artifact: Chapter 5 – ROS 2 Communication Primitives: Nodes, Topics, Services, and Actions..."

## Learner Scenarios & Testing *(mandatory)*

### Learner Story 1 - Building a Mental Model of ROS 2 Communication (Priority: P1)

A student who understands the high-level architecture of ROS 2 now needs to learn the fundamental communication building blocks. They read Chapter 5 to understand what nodes, topics, services, and actions are, and more importantly, when to use each one in the context of building a humanoid robot's software system.

**Why this priority**: This chapter provides the essential vocabulary and mental model for designing and implementing any ROS 2 system. Mastering these concepts is a non-negotiable prerequisite for writing any ROS 2 code.

**Independent Test**: The student's understanding can be tested with design scenarios. After reading the chapter, a learner should be able to take a simple robotic task (e.g., "fetch an object when a button is pressed") and correctly design a ROS 2 graph for it, justifying their choice of topics, services, or actions for each communication channel.

**Acceptance Scenarios**:

1.  **Given** a student has completed Chapter 5, **When** asked to design a system for streaming camera data to a processing node, **Then** they correctly choose to use a topic and can explain why its asynchronous, one-to-many nature is suitable.
2.  **Given** the same student, **When** asked to design a mechanism to trigger a robot's self-calibration routine and wait for it to complete, **Then** they correctly choose a service and can explain why its synchronous, request-response model is appropriate.
3.  **Given** the same student, **When** asked to design a system for commanding a robot to walk to a specific coordinate, providing updates along the way, **Then** they correctly choose an action and can explain how it handles long-running, preemptible tasks with feedback.

---

### Edge Cases

-   **Choosing the Wrong Primitive**: What if a learner uses a service for high-frequency data streaming? The chapter must include a clear comparative decision table (Section 8) and explicit design patterns to guide learners away from common anti-patterns.
-   **Conceptual vs. Practical Confusion**: How does the chapter separate the concepts from the code? It must use minimal, illustrative Python examples purely to ground the concepts, while explicitly stating that implementation details will follow in the next chapter.

## Requirements *(mandatory)*

### Content & Structural Requirements

-   **FR-001**: The chapter MUST teach the four core ROS 2 communication primitives: Nodes, Topics, Services, and Actions.
-   **FR-002**: The chapter MUST be structured into the twelve specified sections, from "Chapter Overview" to "Conceptual Exercises & Design Scenarios".
-   **FR-003**: The chapter MUST explain ROS 2 Nodes as the fundamental unit of computation and the concept of the ROS 2 graph.
-   **FR-004**: The chapter MUST detail the three main communication patterns: Topics (asynchronous), Services (synchronous request-response), and Actions (long-running, goal-oriented tasks).
-   **FR-005**: The chapter MUST provide a clear comparative analysis of when to use Topics vs. Services vs. Actions.
-   **FR-006**: The chapter MUST conceptually introduce message definitions (.msg) and the importance of interface design.
-   **FR-007**: The chapter MUST discuss naming conventions and the high-level concept of composition.
-   **FR-008**: The chapter MUST have a target length of 3,500–4,500 words.
-   **FR-009**: The content MUST use minimal, beginner-friendly Python (rclpy) examples for illustration only.
-   **FR-010**: All technical terminology MUST be defined and used consistently with previous chapters.

### Visual Requirements

-   **FR-011**: The chapter MUST include ROS 2 graph diagrams to visualize communication patterns.
-   **FR-012**: The chapter MUST include a comparative diagram or table illustrating the differences between Topics, Services, and Actions.
-   **FR-013**: The chapter MUST include visuals of a sensor-to-actuator pipeline mapped onto ROS 2 primitives.

## Assumptions

-   Learners have a solid conceptual understanding of ROS 2 architecture from Chapter 4 and basic Python skills.
-   This chapter focuses on the "what" and "why" of communication primitives. The "how" (detailed implementation) will be the focus of the next chapter.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: After reading the chapter, 95% of learners can correctly choose the appropriate communication primitive (Topic, Service, or Action) for five different robotics-related design scenarios.
-   **SC-002**: After reading the chapter, learners can sketch a valid ROS 2 graph for a simple humanoid robot task involving at least three nodes and two different communication types.
-   **SC-003**: The chapter is a direct prerequisite for the next chapter on implementation, with learners reporting high confidence in their ability to apply these concepts in code.
-   **SC-004**: In subsequent modules, learners demonstrate a strong ability to justify their communication design choices in their projects, referencing the principles from this chapter.
