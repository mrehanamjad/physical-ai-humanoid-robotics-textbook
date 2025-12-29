# Tasks: Chapter 12 – Robot Description Formats (SDF vs. URDF)

**Input**: Design documents from `/specs/013-chapter12-robot-description-formats/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. This chapter is primarily conceptual.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

The chapter file will be created at `textbook/docs/module2/chapter12-robot-description-formats.mdx`.

## Phase 1: Setup (Shared Infrastructure)

- [ ] T001 Create the chapter file `textbook/docs/module2/chapter12-robot-description-formats.mdx`.

---

## Phase 2: User Story 1 - Conceptual Differentiation (Priority: P1) 🎯 MVP

**Goal**: A learner reads the chapter to build a clear mental model of the differences between URDF and SDF, focusing on their design goals and intended use cases.

**Independent Test**: The learner can, without referring to notes, list two key strengths of URDF and two key strengths of SDF.

### Implementation for User Story 1

- [ ] T002 [US1] Write the "Chapter Overview" section in `textbook/docs/module2/chapter12-robot-description-formats.mdx`.
- [ ] T003 [US1] Write the "What Is a Robot Description Format?" section in the chapter file.
- [ ] T004 [US1] Write the "URDF (Unified Robot Description Format)" section, detailing its components and limitations.
- [ ] T005 [US1] Write the "SDF (Simulation Description Format)" section, highlighting its additional capabilities for simulation.
- [ ] T006 [US1] Write the "URDF vs SDF: Conceptual Comparison" section, including a detailed feature-comparison table.
- [ ] T007 [US1] Create and embed the "Side-by-Side URDF vs. SDF Structure" diagram in the chapter file.

**Checkpoint**: User Story 1 is complete. The core differences between URDF and SDF are explained.

---

## Phase 3: User Story 2 - Format Selection (Priority: P2)

**Goal**: A learner is presented with several robotics project scenarios and must decide whether to model the robot in URDF, SDF, or a hybrid approach.

**Independent Test**: The learner can justify their choice of format for a given project based on its requirements.

### Implementation for User Story 2

- [ ] T008 [US2] Write the "Modeling Humanoid Robots: Practical Guidelines" section in the chapter file, providing a decision framework for format selection.

**Checkpoint**: User Story 2 is complete. Learners are equipped to choose the correct format.

---

## Phase 4: User Story 3 - Workflow Understanding (Priority: P3)

**Goal**: A learner traces the flow of a URDF model from its file on disk to being a simulated entity in Gazebo, understanding the conceptual conversion to SDF.

**Independent Test**: The learner can draw a simple block diagram showing the URDF-to-SDF pipeline.

### Implementation for User Story 3

- [ ] T009 [US3] Write the "URDF and SDF in Gazebo" section, explaining the on-the-fly conversion and the role of the `<gazebo>` tag.
- [ ] T010 [US3] Write the "URDF and ROS 2 Integration" section, clarifying the "URDF for ROS, SDF for Gazebo" motto.
- [ ] T011 [US3] Create and embed the "Robot Description Pipeline (URDF → Gazebo → SDF)" diagram in the chapter file.

**Checkpoint**: User Story 3 is complete. The toolchain and workflow are demystified.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the chapter with supporting materials and review.

- [ ] T012 Write the "Common Modeling Mistakes" section in the chapter file.
- [ ] T013 Write the "Summary, Key Takeaways, and Practice Questions" section.
- [ ] T014 Review the entire chapter for technical accuracy, clarity, and consistency with previous chapters (Ch. 8, 10, 11).
- [ ] T015 Validate that all diagrams are clear, correct, and reinforce the text effectively.

---

## Dependencies & Execution Order

- **Setup (Phase 1)**: Must be completed first.
- **User Stories (Phase 2-4)**: Should be completed in order (US1 → US2 → US3) as the concepts are progressive.
- **Polish (Phase 5)**: Depends on all user stories being complete.

## Implementation Strategy

The implementation will follow the task order to build the chapter's narrative logically.

1.  Complete Phase 1 (Setup).
2.  Complete Phase 2 (US1) to establish the core conceptual differences.
3.  Complete Phase 3 (US2) to provide practical decision-making guidance.
4.  Complete Phase 4 (US3) to explain the software toolchain.
5.  Complete Phase 5 (Polish) for final review and knowledge validation.
