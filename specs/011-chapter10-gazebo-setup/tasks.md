# Tasks: Chapter 10 – Gazebo Environment Setup

**Input**: Design documents from `/specs/011-chapter10-gazebo-setup/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. This chapter is a tutorial, so tasks represent sections of the document.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

The chapter file will be created at `textbook/docs/module2/chapter10-gazebo-setup.mdx`.

## Phase 1: Setup (Shared Infrastructure)

- [ ] T001 Create the chapter file `textbook/docs/module2/chapter10-gazebo-setup.mdx`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Write the introductory content that sets the stage for the hands-on tutorial.

- [ ] T002 Write the "Chapter Overview" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.
- [ ] T003 Write the "Gazebo in the Robotics Ecosystem" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.
- [ ] T004 Write the "Choosing the Right Gazebo Version" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`, recommending Gazebo Harmonic with ROS 2 Iron.

**Checkpoint**: Foundational concepts are explained.

---

## Phase 3: User Story 1 - Environment Setup (Priority: P1) 🎯 MVP

**Goal**: A learner follows the chapter instructions to install and configure Gazebo and its ROS 2 integration packages on a clean system.

**Independent Test**: The learner can successfully launch the Gazebo GUI and verify the installation of the `ros_gz_bridge` package.

### Implementation for User Story 1

- [ ] T005 [US1] Write the "Installing Gazebo and Integration" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx` with copy-paste friendly commands.
- [ ] T006 [US1] Add verification steps to confirm successful installation in the "Installing Gazebo and Integration" section.
- [ ] T007 [US1] Write the "Gazebo Workspace and Project Structure" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.

**Checkpoint**: User Story 1 is complete. A learner can set up their environment.

---

## Phase 4: User Story 2 - Simulation Launch (Priority: P2)

**Goal**: A learner uses a ROS 2 launch file to start a Gazebo simulation and load a provided humanoid robot model (URDF).

**Independent Test**: The learner can execute a single `ros2 launch` command that opens Gazebo and displays the robot model.

### Implementation for User Story 2

- [ ] T008 [US2] Write the "Launching a Basic Gazebo Simulation" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.
- [ ] T009 [US2] Write the "Loading Robots into Gazebo" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`, including a sample launch file.
- [ ] T010 [US2] Create and embed the "Gazebo-ROS 2 Integration Architecture" diagram in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.
- [ ] T011 [US2] Create a simple world file (e.g., `empty.sdf`) in `textbook/code/chapter10/worlds/`
- [ ] T012 [US2] Create the example launch file (e.g., `spawn_robot.launch.py`) in `textbook/code/chapter10/launch/`

**Checkpoint**: User Story 2 is complete. A learner can launch a simulation with a robot.

---

## Phase 5: User Story 3 - Communication Validation (Priority: P3)

**Goal**: A learner validates the bidirectional communication between ROS 2 and Gazebo by inspecting sensor data and sending a basic command.

**Independent Test**: The learner can `echo` a ROS 2 topic to see live sensor data from the simulated robot and `pub` a message to make the robot move.

### Implementation for User Story 3

- [ ] T013 [US3] Write the "Validating ROS 2 ↔ Gazebo Communication" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx` with `ros2 topic echo` and `ros2 topic pub` examples.

**Checkpoint**: User Story 3 is complete. The digital twin communication is verified.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the chapter with supporting materials and review.

- [ ] T014 Write the "Common Setup Issues" troubleshooting section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.
- [ ] T015 Write the "Summary, Key Takeaways, and Practice Questions" section in `textbook/docs/module2/chapter10-gazebo-setup.mdx`.
- [ ] T016 Perform a full, clean-room test of the entire tutorial from start to finish on a fresh ROS 2 installation to ensure 100% reproducibility.
- [ ] T017 Review the entire chapter for clarity, accuracy, and grammatical correctness.

---

## Dependencies & Execution Order

- **Setup & Foundational (Phase 1-2)**: Must be completed first.
- **User Stories (Phase 3-5)**: Must be completed in order (US1 → US2 → US3) as they build on each other.
- **Polish (Phase 6)**: Depends on all user stories being complete.

## Implementation Strategy

This is a sequential tutorial. The implementation must follow the task order precisely to ensure the narrative flow of the chapter is logical for the learner.

1.  Complete Phase 1 & 2 (Setup & Foundational).
2.  Complete Phase 3 (US1) → Test installation steps.
3.  Complete Phase 4 (US2) → Test simulation launch.
4.  Complete Phase 5 (US3) → Test communication.
5.  Complete Phase 6 (Polish) → Final review and clean-room validation.
