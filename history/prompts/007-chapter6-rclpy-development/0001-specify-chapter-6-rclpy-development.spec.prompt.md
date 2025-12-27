---
id: 0001
title: Specify Chapter 6 rclpy Development
stage: spec
date: 2025-12-27
surface: agent
model: gemini-pro
feature: 007-chapter6-rclpy-development
branch: 007-chapter6-rclpy-development
user:
command: "Artifact: Chapter 6 – Python-Based ROS 2 Development with rclpy..."
labels: ["spec", "chapter6", "ros2", "rclpy"]
links:
  spec: specs/007-chapter6-rclpy-development/spec.md
  ticket:
  adr:
  pr:
files:
  - specs/007-chapter6-rclpy-development/spec.md
  - specs/007-chapter6-rclpy-development/checklists/requirements.md
tests:

---

## Prompt

Artifact: Chapter 6 – Python-Based ROS 2 Development with rclpy Module: Module 1: The Robotic Nervous System (ROS 2) Scope: Specify the complete instructional content, structure, and pedagogy for Chapter 6 only. This chapter introduces practical ROS 2 development using Python (rclpy). Do NOT include chatbot systems, web frameworks, databases, or non-ROS tooling. Chapter purpose: Enable learners to implement ROS 2 nodes in Python using rclpy, translating the conceptual communication primitives (nodes, topics, services, actions) into executable robot software. This chapter forms the coding foundation for all future modules. Target audience: Students who: - Understand ROS 2 concepts (Chapters 4–5) - Know basic Python programming - Are new to ROS 2 development workflows Prerequisites: - Chapter 4: ROS 2 Architecture and Core Concepts - Chapter 5: Nodes, Topics, Services, and Actions - Basic Python syntax and OOP concepts Learning objectives: By the end of this chapter, learners should be able to: - Understand the role of rclpy in the ROS 2 ecosystem - Create and structure Python-based ROS 2 nodes - Implement publishers and subscribers - Implement simple services and clients - Implement basic actions (conceptual + minimal code) - Run, inspect, and reason about ROS 2 nodes they write - Follow ROS 2 Python best practices Chapter layout requirements: The chapter must be structured with the following sections: 1. Chapter Overview - From communication theory to executable code - Why Python is ideal for AI-driven robotics 2. Introducing rclpy - What is rclpy? - Relationship between ROS 2 core (rcl) and Python bindings - When to use Python vs C++ in robotics 3. Anatomy of a Python ROS 2 Node - Node lifecycle - Initialization and shutdown - Spinning and callbacks - Single-responsibility node design 4. Creating a Publisher Node - Publishing data to a topic - Timer-based publishers - Example: publishing joint states or sensor values - Common mistakes and debugging intuition 5. Creating a Subscriber Node - Subscribing to topics - Callback execution model - Message handling and safety - Example: consuming IMU or camera metadata 6. Working with Parameters in rclpy - Declaring and retrieving parameters - Dynamic configuration at runtime - Use cases in humanoid robots 7. Services in rclpy - Creating a service server - Creating a service client - Blocking vs asynchronous calls - Example: mode switching or configuration queries 8. Actions in rclpy - Conceptual overview of Python actions - Action servers and clients - Feedback and result handling - Example: navigation or motion goal execution 9. Logging, Debugging, and Introspection - ROS 2 logging levels - Console tools (conceptual only) - Debugging callback-based systems 10. Coding Patterns for Humanoid Robots - Sensor-processing nodes - Control and command nodes - Decoupling AI logic from hardware interfaces 11. Performance Considerations - Python limitations and strengths - When to refactor to C++ - Managing callbacks and timers responsibly 12. Chapter Summary and Key Takeaways - rclpy mental model recap - Preparing for ROS 2 package development 13. Hands-On Exercises - Build a publisher–subscriber pair - Implement a simple service - Design an action-based task Content standards: - Use Python (rclpy) exclusively - Code must be minimal, readable, and beginner-safe - No installation or OS-specific setup steps - Avoid deep QoS configuration (introduced later) - Avoid heavy math or control theory - All examples must be humanoid-robot–relevant Code quality requirements: - Follow ROS 2 Python style conventions - Clear function naming and comments - No hidden “magic” code - Explicit explanation of callbacks and execution flow Visual requirements: - Node lifecycle diagrams - Callback flow diagrams - Publisher–subscriber interaction diagrams Writing style: - Practical, calm, and confidence-building - Explain “what happens when this runs” - Reinforce ROS mental models continuously - Assume curiosity, not expertise Length constraints: - Target length: 4,000–5,000 words Success criteria: - Learners can write and run basic ROS 2 Python nodes - Learners understand how their code maps to the ROS 2 graph - Learners are ready to build full ROS 2 packages next - Chapter directly supports Gazebo and Isaac integration in later modules

## Response snapshot

I have created the feature branch `007-chapter6-rclpy-development` and the corresponding specification file at `../specs/007-chapter6-rclpy-development/spec.md`.

The specification is based on the detailed description you provided. I have also created a validation checklist at `../specs/007-chapter6-rclpy-development/checklists/requirements.md`, and all checks have passed.

The feature is now ready for the next phase, which can be either planning (`/sp.plan`) or clarification (`/sp.clarify`).

## Outcome

- ✅ Impact: Created a new specification for a feature.
- 🧪 Tests: No tests were run.
- 📁 Files:
  - ../specs/007-chapter6-rclpy-development/spec.md
  - ../specs/007-chapter6-rclpy-development/checklists/requirements.md
- 🔁 Next prompts: /sp.plan or /sp.clarify
- 🧠 Reflection: The process is now well-established and smooth.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable):
- Next experiment (smallest change to try):
