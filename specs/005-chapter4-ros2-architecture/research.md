---
title: "Research Log: Chapter 4 – ROS 2 Architecture"
feature: "005-chapter4-ros2-architecture"
status: "draft"
date: "2025-12-27"
---

## 1. Research Questions

This document tracks the research needed to fulfill the requirements of `plan.md` for Chapter 4.

### Foundational Research (Phase 1)

1.  **Official Documentation**: What are the key points and diagrams in the official ROS 2 documentation's "Concepts" and "Design" sections? This is the primary source of truth.
    - *Source*: `docs.ros.org`

2.  **Beginner-Friendly Explanations**: What are the best community-produced explanations (articles, videos) of the ROS 2 architecture?
    - *Keywords*: "ROS 2 architecture explained", "ROS 2 for beginners", "ROS 2 vs ROS 1".
    - *Sources*: The Robotics Stack Exchange, Medium articles, conference talks (e.g., ROSCon).

3.  **DDS Analogies**: What are the most effective analogies for explaining DDS to someone with no prior knowledge of industrial middleware?
    - *Examples*: "Shared whiteboard", "magic phonebook", "data-centric bus".

### Just-in-Time Research (Phase 2)

- **ROS 1 vs ROS 2**: What are the 2-3 most critical improvements of ROS 2 over ROS 1 that are relevant to a beginner? (e.g., no `roscore`, DDS integration, better real-time support).
- **Executor Model**: How can the role of an executor be explained simply but accurately? Find a good analogy (e.g., "event loop", "callback scheduler").
- **Common Sticking Points**: What are the most common conceptual hurdles for learners new to ROS 2? (e.g., understanding that a topic is a "concept" and not a "pipe", the node-vs-process distinction).

## 2. Research Findings

*(This section will be populated as research is conducted.)*

### Decision: Architectural Analogy - OSI Model

- **Rationale**: A layered stack is a familiar concept to most developers and computer science students through the OSI or TCP/IP networking models. Comparing the ROS 2 stack (Hardware -> OS -> DDS -> RMW -> RCL -> User Code) to the networking stack provides a powerful mental model for understanding separation of concerns.
- **Alternatives Considered**:
    - **No Analogy**: Simply presenting the layers was deemed too abstract and less memorable.
    - **Custom Analogy**: Inventing a new analogy (e.g., a "corporate hierarchy") was considered more confusing than leveraging a well-known existing model.

### References

1.  [ROS 2 Design Articles](https://design.ros2.org/)
2.  [ROS 2 Documentation: Concepts](https://docs.ros.org/en/rolling/Concepts.html)
