# Tasks: Chapter 6 – Python-Based ROS 2 Development with rclpy

**Input**: Design documents from `/specs/007-chapter6-rclpy-development/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)

## Path Conventions

- All paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the primary file for the chapter.

- [X] T001 Create the chapter markdown file in `textbook/docs/module1-ros2/06-python-ros2-development.md`.
- [X] T002 Create a directory for code examples for this chapter at `textbook/code/chapter6/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Complete the necessary research before writing begins.

- [X] T003 [P] Research beginner-safe `rclpy` APIs for nodes, publishers, subscribers, services, and actions.
- [X] T004 [P] Research clear ways to explain the `rclpy` execution model (spinning, callbacks) to beginners.
- [X] T005 [P] Research common `rclpy` coding patterns and performance considerations.
- [X] T006 Consolidate research findings into `specs/007-chapter6-rclpy-development/research.md`.

**Checkpoint**: Foundation ready - chapter writing and code implementation can now begin.

---

## Phase 3: User Story 1 - From Theory to Code (Priority: P1) 🎯 MVP

**Goal**: To write a chapter that teaches the practical skills of writing, running, and inspecting basic ROS 2 nodes using the `rclpy` library.

**Independent Test**: A reader, after completing the chapter, can independently write and run a functional publisher/subscriber pair and a service/client pair.

### Implementation for User Story 1

- [X] T007 [US1] Write the "Chapter Overview" section in `textbook/docs/module1-ros2/06-python-ros2-development.md`.
- [X] T008 [US1] Write the "Anatomy of a Python ROS 2 Node" section, explaining initialization, spinning, and callbacks, in `textbook/docs/module1-ros2/06-python-ros2-development.md`.
- [X] T009 [P] [US1] Create a diagram illustrating the lifecycle of a ROS 2 node and save it to `textbook/static/img/ch06-node-lifecycle.png`.
- [X] T010 [P] [US1] Create a diagram showing callback data flow and save it to `textbook/static/img/ch06-callback-flow.png`.
- [X] T011 [US1] Write the "Creating a Publisher Node" section. Create the corresponding example code in `textbook/code/chapter6/publisher_node.py`.
- [X] T012 [US1] Write the "Creating a Subscriber Node" section. Create the corresponding example code in `textbook/code/chapter6/subscriber_node.py`.
- [X] T013 [US1] Write the "Running Your First Nodes" section, explaining how to run the publisher/subscriber pair.
- [X] T014 [US1] Write the "Creating a Service Server" section. Create the corresponding example code in `textbook/code/chapter6/service_server.py`.
- [X] T015 [US1] Write the "Creating a Service Client" section. Create the corresponding example code in `textbook/code/chapter6/service_client.py`.
- [X] T016 [US1] Write the "Creating an Action Server" section (conceptual with minimal code). Create example in `textbook/code/chapter6/action_server.py`.
- [X] T017 [US1] Write the "Creating an Action Client" section (conceptual with minimal code). Create example in `textbook/code/chapter6/action_client.py`.
- [X] T018 [US1] Write the "Using Parameters in `rclpy`" section. Create a corresponding example in `textbook/code/chapter6/parameter_node.py`.
- [X] T019 [US1] Write the "Logging and Debugging" section.
- [X] T020 [US1] Write the "Common Coding Patterns & Performance" section.
- [X] T021 [US1] Integrate all diagrams (T009, T010) and code examples into the chapter markdown file.
- [X] T022 [US1] Write the "Hands-On Exercises" section, including exercises for writing a pub/sub system and a service/client system.

**Checkpoint**: At this point, User Story 1 should be fully implemented. The chapter and its code examples are ready for review.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation that affect the entire chapter.

- [X] T023 Perform a full review of `textbook/docs/module1-ros2/06-python-ros2-development.md` for clarity, technical accuracy, and grammatical correctness.
- [ ] T024 Test all code examples in `textbook/code/chapter6/` to ensure they are runnable and function as described.
- [X] T025 Validate that all content requirements from `specs/007-chapter6-rclpy-development/spec.md` have been met.
- [X] T026 Check for consistent use of terminology with previous chapters and the book's glossary.
- [X] T027 Ensure all diagrams and code snippets are correctly referenced and clearly explained in the text.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion.
- **User Story 1 (Phase 3)**: Depends on Foundational phase completion.
- **Polish (Phase 4)**: Depends on User Story 1 completion.

### User Story Dependencies

- **User Story 1 (P1)**: The only user story for this feature.

### Within User Story 1

- Research tasks (T003-T005) should be completed before writing begins.
- Diagram creation (T009, T010) can happen in parallel with writing tasks but must be completed before the integration task (T021).
- Writing and coding tasks (T007-T020) should proceed in the specified order as they build upon each other.

### Parallel Opportunities

- **T003**, **T004**, and **T005** can be performed in parallel.
- **T009** and **T010** (diagram creation) can be performed in parallel with the writing tasks.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  Complete Phase 4: Polish.
5.  **STOP and VALIDATE**: Review the complete chapter and test all code examples.
