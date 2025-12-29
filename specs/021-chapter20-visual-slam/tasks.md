# Tasks: Chapter 20 – Visual SLAM

**Input**: Design documents from `/specs/021-chapter20-visual-slam/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. This chapter combines foundational concepts with a major hands-on tutorial.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

The chapter file will be created at `textbook/docs/module3/chapter20-visual-slam.mdx`. Supporting scripts will be in `textbook/code/chapter20/`.

## Phase 1: Setup (Shared Infrastructure)

- [ ] T001 Create the chapter file `textbook/docs/module3/chapter20-visual-slam.mdx`.
- [ ] T002 Create the directory `textbook/code/chapter20/` for evaluation scripts.

---

## Phase 2: User Story 1 - Conceptual Understanding (Priority: P1)

**Goal**: A learner understands the core "chicken-and-egg" problem of SLAM and its main algorithmic components.

**Independent Test**: The learner can explain the roles of localization, mapping, and loop closure.

### Implementation for User Story 1

- [ ] T003 [US1] Write the "Chapter Overview" section in the chapter file, introducing SLAM as the foundational capability for autonomy.
- [ ] T004 [US1] Write the "Fundamentals of Visual SLAM" section, explaining tracking, mapping, and loop closure with clear, intuitive language.
- [ ] T005 [US1] Create and embed a high-level diagram of the complete "Visual SLAM Data Pipeline."
- [ ] T006 [US1] Create and embed a diagram illustrating a "Pose Graph Before and After Loop Closure" to visually explain the optimization process.

**Checkpoint**: User Story 1 is complete. The conceptual framework of SLAM is established.

---

## Phase 3: User Story 2 - Implementing a SLAM Pipeline (Priority: P2) 🎯 MVP

**Goal**: A learner can launch a complete, hardware-accelerated Visual SLAM pipeline using Isaac ROS in a simulated environment.

**Independent Test**: The learner can drive a simulated robot and see a map being built in real-time in RViz.

### Implementation for User Story 2

- [ ] T007 [US2] Write the "Hands-On: Mapping Your World" tutorial section in the chapter file.
- [ ] T008 [US2] Sub-task: Provide step-by-step instructions for launching the Isaac ROS Visual SLAM Docker container and the main launch file.
- [ ] T009 [US2] Sub-task: Guide the learner on how to drive the robot in Isaac Sim to thoroughly map a pre-defined environment.
- [ ] T010 [US2] Sub-task: Provide detailed instructions on configuring RViz to visualize all the necessary outputs: camera feed, robot pose, point cloud map, and trajectory.

**Checkpoint**: User Story 2 is complete. The learner has successfully generated a map using a high-performance SLAM system.

---

## Phase 4: User Story 3 - Evaluating SLAM Performance (Priority: P3)

**Goal**: A learner can use tools to evaluate the quality of the map and the accuracy of the trajectory from their SLAM pipeline.

**Independent Test**: The learner can calculate the Absolute Trajectory Error (ATE) of a SLAM run against ground truth.

### Implementation for User Story 3

- [ ] T011 [US3] Write the "Is My Map Any Good? Evaluating SLAM" tutorial section.
- [ ] T012 [US3] Sub-task: Explain the concept of "ground truth" in simulation and why it's crucial for evaluation.
- [ ] T013 [US3] Sub-task: Provide a step-by-step guide on how to record the SLAM trajectory and the ground truth trajectory.
- [ ] T014 [US3] Sub-task: Provide a Python script in `textbook/code/chapter20/evaluate_slam.py` and instructions on how to use it to calculate and interpret the Absolute Trajectory Error (ATE).

**Checkpoint**: User Story 3 is complete. The learner can quantitatively measure the performance of their SLAM system.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the chapter with supporting materials and perform validation.

- [ ] T015 Write the "Common Failure Modes" section, showing visual examples of what happens with poor sensor data or in textureless environments.
- [ ] T016 Write the "Summary & Next Steps" section, connecting SLAM to the upcoming Navigation chapter.
- [ ] T017 **(CRITICAL)** Perform a full end-to-end test of the tutorials in Phase 3 and 4 to guarantee reproducibility.
- [ ] T018 Review the entire chapter for technical accuracy, clarity of instructions, and conceptual correctness.

---

## Dependencies & Execution Order

- **Setup (Phase 1)**: Must be completed first.
- **User Stories (Phase 2-4)**: Must be completed in order (US1 → US2 → US3) as they represent a continuous workflow from theory to practice to analysis.
- **Polish (Phase 5)**: Depends on all user stories being complete.

## Implementation Strategy

The implementation will follow the task order to build a practical and conceptually sound chapter on Visual SLAM.

1.  Complete Phase 1 (Setup).
2.  Complete Phase 2 (US1) to build the conceptual model of SLAM.
3.  Complete Phase 3 (US2) for the core hands-on skill of running a SLAM system.
4.  Complete Phase 4 (US3) to add the essential engineering skill of performance evaluation.
5.  Complete Phase 5 (Polish) to finalize and validate the chapter.
