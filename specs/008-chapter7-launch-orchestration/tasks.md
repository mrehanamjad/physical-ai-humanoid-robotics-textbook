# Tasks: Chapter 7 – Launch Files, Parameters, and System Orchestration

**Input**: Design documents from `/specs/008-chapter7-launch-orchestration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)

## Path Conventions

- All paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the primary file for the chapter and its associated code examples.

- [X] T001 Create the chapter markdown file in `textbook/docs/module1-ros2/07-launch-files-and-parameters.md`.
- [X] T002 Create a directory for code examples for this chapter at `textbook/code/chapter7/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Complete the necessary research before writing begins.

- [X] T003 [P] Research the relationship between Python launch files, the `launch` library, and ROS 2 CLI tools.
- [X] T004 [P] Research the ROS 2 parameter system to define an appropriate depth for beginners.
- [X] T005 [P] Research lifecycle nodes to determine the scope of their conceptual introduction.
- [X] T006 Consolidate research findings into a new `research.md` file in `specs/008-chapter7-launch-orchestration/`.

**Checkpoint**: Foundation ready - chapter writing and code implementation can now begin.

---

## Phase 3: User Story 1 - From Individual Nodes to a Coordinated System (Priority: P1) 🎯 MVP

**Goal**: To teach learners how to use Python-based launch files to start, configure, and orchestrate multiple ROS 2 nodes, moving them from a node-level to a system-level mindset.

**Independent Test**: A reader, after completing the chapter, can write a single launch file to start a multi-node application and configure it using a YAML parameter file.

### Implementation for User Story 1

- [X] T007 [US1] Write the "Chapter Overview" section in `textbook/docs/module1-ros2/07-launch-files-and-parameters.md`.
- [X] T008 [US1] Write the "Why Launch Files?" section, motivating the need for orchestration, in `textbook/docs/module1-ros2/07-launch-files-and-parameters.md`.
- [X] T009 [US1] Write the "Anatomy of a Python Launch File" section, explaining `LaunchDescription` and `Node` actions, in `textbook/docs/module1-ros2/07-launch-files-and-parameters.md`.
- [X] T010 [US1] Write the "Launching a Single Node" section. Create the corresponding example `simple_launch.py` in `textbook/code/chapter7/`.
- [X] T011 [US1] Write the "Launching Multiple Nodes" section. Create `multi_node_launch.py` in `textbook/code/chapter7/`.
- [X] T012 [P] [US1] Create a system orchestration diagram showing a launch file starting multiple nodes and save it to `textbook/static/img/ch07-orchestration.png`.
- [X] T013 [US1] Write the "Introducing Parameters" section in `textbook/docs/module1-ros2/07-launch-files-and-parameters.md`.
- [X] T014 [US1] Write the "Using YAML for Parameters" section. Create an example `params.yaml` and a `param_launch.py` in `textbook/code/chapter7/`.
- [X] T015 [US1] Write the "Namespaces for Multi-Robot Systems" section. Create `namespaced_launch.py` in `textbook/code/chapter7/`.
- [X] T016 [P] [US1] Create a diagram showing a namespace hierarchy and save it to `textbook/static/img/ch07-namespaces.png`.
- [X] T017 [US1] Write the "Remapping Topics" section. Create `remap_launch.py` in `textbook/code/chapter7/`.
- [X] T018 [US1] Write the "Including Other Launch Files" section.
- [X] T019 [US1] Write the "Lifecycle (Managed) Nodes: A Conceptual Introduction" section.
- [X] T020 [P] [US1] Create a diagram illustrating a node startup sequence and save it to `textbook/static/img/ch07-startup-sequence.png`.
- [X] T021 [US1] Write the "Debugging Launch Failures" section.
- [X] T022 [US1] Write the "Best Practices for Scalable Launch Files" section.
- [X] T023 [US1] Integrate all diagrams (T012, T016, T020) into the chapter markdown file.
- [X] T024 [US1] Write the "Hands-On Exercises" section, including exercises for launching a multi-node system with parameters.

**Checkpoint**: At this point, User Story 1 should be fully implemented. The chapter and its code examples are ready for review.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation that affect the entire chapter.

- [X] T025 Perform a full review of `textbook/docs/module1-ros2/07-launch-files-and-parameters.md` for clarity, technical accuracy, and grammatical correctness.
- [ ] T026 Test all launch files in `textbook/code/chapter7/` to ensure they function as described.
- [X] T027 Validate that all content requirements from `specs/008-chapter7-launch-orchestration/spec.md` have been met.
- [X] T028 Check for consistent use of terminology with previous chapters and the book's glossary.
- [X] T029 Ensure all diagrams and code snippets are correctly referenced and clearly explained in the text.

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
- Diagram creation can happen in parallel with writing tasks but must be completed before the integration task (T023).
- Writing and coding tasks (T007-T024, excluding diagram integration) should proceed in the specified order as they build upon each other.

### Parallel Opportunities

- **T003**, **T004**, and **T005** can be performed in parallel.
- Diagram creation tasks (**T012**, **T016**, **T020**) can be performed in parallel with the writing tasks.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  Complete Phase 4: Polish.
5.  **STOP and VALIDATE**: Review the complete chapter and test all code examples.
