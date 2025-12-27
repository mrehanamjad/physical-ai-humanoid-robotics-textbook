---
id: 0001
title: Specify Chapter 5 ROS 2 Primitives
stage: spec
date: 2025-12-27
surface: agent
model: gemini-pro
feature: 006-chapter5-ros2-primitives
branch: 006-chapter5-ros2-primitives
user:
command: "Artifact: Chapter 5 – ROS 2 Communication Primitives: Nodes, Topics, Services, and Actions..."
labels: ["spec", "chapter5", "ros2", "primitives"]
links:
  spec: specs/006-chapter5-ros2-primitives/spec.md
  ticket:
  adr:
  pr:
files:
  - specs/006-chapter5-ros2-primitives/spec.md
  - specs/006-chapter5-ros2-primitives/checklists/requirements.md
tests:

---

## Prompt

Artifact: Chapter 5 – ROS 2 Communication Primitives: Nodes, Topics, Services, and Actions Module: Module 1: The Robotic Nervous System (ROS 2) Scope: Specify the full instructional content, structure, and pedagogical design for Chapter 5 only. This chapter introduces the core ROS 2 communication abstractions that enable distributed robot behavior. Do NOT include chatbot systems, web backends, databases, or non-robotics infrastructure. Chapter purpose: Teach learners how computation and communication occur inside a ROS 2 system by explaining nodes, topics, services, and actions both conceptually and practically. This chapter is the foundation for all future robot behavior, perception, and control pipelines. Target audience: Students who: - Understand ROS 2 architecture at a conceptual level (Chapter 4) - Are new to ROS 2 programming - Will soon build ROS 2 packages using Python (rclpy) Prerequisites: - Chapter 4: ROS 2 Architecture and Core Concepts - Basic Python familiarity Learning objectives: By the end of this chapter, learners should be able to: - Explain what a ROS 2 node is and why nodes are modular - Understand how topics enable asynchronous data flow - Understand services as request/response interactions - Understand actions for long-running, goal-oriented tasks - Choose the correct communication primitive for a given robotic task - Read and reason about ROS 2 graphs conceptually Chapter layout requirements: The chapter must be structured with the following sections: 1. Chapter Overview - From architecture to behavior - How ROS 2 primitives form the robot’s communication fabric 2. Nodes: The Fundamental Unit of Computation - What is a node? - Node lifecycle and responsibilities - Single-responsibility principle in robotics - Examples: sensor node, controller node, planner node - Nodes in humanoid robots 3. The ROS 2 Graph - Nodes and edges - Runtime discovery - Decentralization and fault tolerance - Visualizing robot behavior as a graph 4. Topics: Asynchronous Data Streams - Publish/subscribe model - Message flow and decoupling - Typical topic use cases (sensor data, state updates) - Topic frequency and data rates (conceptual) - Humanoid examples: camera images, joint states, IMU data 5. Messages and Interfaces - Message definitions (.msg) at a conceptual level - Strong typing and schema evolution - Why message design matters for real robots 6. Services: Request–Response Interactions - Synchronous communication model - When to use services - Blocking vs non-blocking calls (conceptual) - Humanoid examples: mode switching, configuration queries 7. Actions: Long-Running and Goal-Oriented Tasks - Why actions exist - Goals, feedback, and results - Preemption and cancellation - Humanoid examples: walking to a location, picking up an object 8. Topics vs Services vs Actions - Comparative decision table - Latency, duration, reliability, and feedback - Design patterns for robotic systems 9. Communication Patterns in Humanoid Robots - Sensor → perception → planning → control pipelines - Coordinating locomotion and manipulation - Error handling and recovery 10. Naming, Namespaces, and Composition - Node and topic naming conventions - Namespaces for multi-robot systems - Composition concepts (high-level) 11. Chapter Summary and Key Takeaways - Communication primitives recap - Design intuition checklist 12. Conceptual Exercises & Design Scenarios - Choosing the correct communication primitive - Graph design exercises - Fault analysis scenarios Content standards: - Mix of conceptual explanation and illustrative pseudocode - Python (rclpy) examples allowed, but minimal and beginner-friendly - No installation or environment setup steps - No deep QoS tuning (introduced later) - All examples must relate to humanoid robotics Visual requirements: - ROS 2 graph diagrams showing nodes and communication - Comparison diagrams for topics vs services vs actions - Sensor-to-actuator pipeline visuals Writing style: - Learner-focused and structured - Progressive complexity - Strong emphasis on “why” before “how” - Consistent terminology with Chapters 1–4 Length constraints: - Target length: 3,500–4,500 words Success criteria: - Learners can design a ROS 2 graph for a humanoid robot - Learners can justify communication choices - Learners are prepared to implement ROS 2 nodes in the next chapter - Concepts directly support future modules (Gazebo, Isaac, VLA)

## Response snapshot

I have created the feature branch `006-chapter5-ros2-primitives` and the corresponding specification file at `../specs/006-chapter5-ros2-primitives/spec.md`.

The specification is based on the detailed description you provided. I have also created a validation checklist at `../specs/006-chapter5-ros2-primitives/checklists/requirements.md`, and all checks have passed.

The feature is now ready for the next phase, which can be either planning (`/sp.plan`) or clarification (`/sp.clarify`).

## Outcome

- ✅ Impact: Created a new specification for a feature.
- 🧪 Tests: No tests were run.
- 📁 Files:
  - ../specs/006-chapter5-ros2-primitives/spec.md
  - ../specs/006-chapter5-ros2-primitives/checklists/requirements.md
- 🔁 Next prompts: /sp.plan or /sp.clarify
- 🧠 Reflection: The process is now well-established and smooth.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable):
- Next experiment (smallest change to try):
