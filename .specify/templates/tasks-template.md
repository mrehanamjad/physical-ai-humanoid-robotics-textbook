---

description: "Task list template for feature implementation"
---

<!--
**Constitution Reminder:** All tasks MUST be derived from the approved `plan.md` and `spec.md`, and MUST align with the project constitution.
- **Traceability:** Every task should be traceable to a specific requirement or user story.
- **Principle Alignment:** Tasks should reflect the Core Principles (e.g., creating visual aids, writing clear explanations).
- **Standards Compliance:** Tasks must adhere to the Key Standards for code, content, and workflow.
-->

# Tasks: [FEATURE NAME]

**Input**: Design documents from `/specs/[###-feature-name]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

<!-- 
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.
  
  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - The Project Constitution

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Constitution-Driven Task Examples

*The following are examples of how to break down work in compliance with the constitution.*

### Example: Content Creation Task (Chapter 1)
- **[ ] T001 [US1] DOCS:** Draft the "Learning Objectives" for Chapter 1, ensuring they are measurable and align with the progressive learning principle (Const. I.1, II.3).
- **[ ] T002 [US1] DOCS:** Write the main content for the "Introduction to Humanoid Robotics" section, focusing on educational clarity and including relatable analogies (Const. I.5).
- **[ ] T003 [US1] DOCS:** Create a diagram illustrating the main components of a humanoid robot using Excalidraw, following visual design standards (Const. I.4, II.6).
- **[ ] T004 [US1] DOCS:** Add a "Safety" callout box discussing the ethical implications of autonomous humanoid robots (Const. I.6).

### Example: Code Example Task (Chapter 2)
- **[ ] T005 [US2] CODE:** Implement a "hello_robot" Python script demonstrating a basic ROS2 publisher, ensuring it follows PEP 8 and includes type hints (Const. II.2).
- **[ ] T006 [US2] TEST:** Write a unit test for the `add_numbers` utility function used in the script.
- **[ ] T007 [US2] DOCS:** Write a `README.md` for the "hello_robot" example, including setup instructions and expected output (Const. II.2).
- **[ ] T008 [US2] REVIEW:** Perform a peer review of the code and documentation for technical accuracy and clarity (Const. I.3).

### Example: Accessibility Task (Website)
- **[ ] T009 [US3] FE:** Add `alt` text to all images on the homepage, ensuring they are descriptive (Const. II.4).
- **[ ] T010 [US3] FE:** Test the main navigation using only a keyboard to ensure full accessibility (Const. II.4).
- **[ ] T011 [US3] TEST:** Run an automated accessibility check (e.g., Axe) on the deployed site and log any high-priority issues.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan
- [ ] T002 Initialize [language] project with [framework] dependencies
- [ ] T003 [P] Configure linting and formatting tools

---

## Phase 2: User Story 1 - [Title] (Priority: P1) 🎯 MVP

**Goal**: [Brief description of what this story delivers]

### Implementation for User Story 1
- [ ] T012 [P] [US1] Create [Entity1] model in src/models/[entity1].py
- [ ] T013 [US1] Implement [Service] in src/services/[service].py
- [ ] T014 [US1] Implement [endpoint/feature] in src/[location]/[file].py
- [ ] T015 [US1] Add validation and error handling

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.
