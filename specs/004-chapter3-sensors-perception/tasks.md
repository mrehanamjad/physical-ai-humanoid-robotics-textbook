# Tasks: Chapter 3 – Sensors and Perception Basics

**Input**: Design documents from `/specs/004-chapter3-sensors-perception/`
**Prerequisites**: spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)

## Path Conventions

- All paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the primary file for the chapter.

- [X] T001 Create the chapter markdown file in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Complete the necessary research before writing begins.

- [X] T002 [P] Research intuitive explanations for the operating principles of Cameras (RGB, Depth), LiDAR (2D, 3D), and IMUs.
- [X] T003 [P] Research canonical use cases, strengths, and weaknesses for each sensor in a humanoid robotics context.
- [X] T004 Consolidate research findings and key decisions into `specs/004-chapter3-sensors-perception/research.md`.

**Checkpoint**: Foundation ready - chapter writing can now begin.

---

## Phase 3: User Story 1 - Understanding Sensing Modalities (Priority: P1) 🎯 MVP

**Goal**: To write a chapter that explains the fundamental operating principles, strengths, and weaknesses of cameras, LiDAR, and IMUs for humanoid robotics.

**Independent Test**: A reader, after completing the chapter, can correctly choose the most appropriate sensor for a given robotics task and justify their decision.

### Implementation for User Story 1

- [X] T005 [US1] Write the "Chapter Overview" section in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T006 [US1] Write "The Senses of a Robot" section, distinguishing between exteroceptive and proprioceptive sensors, in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T007 [US1] Write the "Cameras: The Robot's Eyes" section, covering RGB and Depth cameras, in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T008 [US1] Write the "LiDAR: Seeing with Light" section, covering 2D and 3D LiDAR, in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T009 [US1] Write the "IMUs: The Sense of Balance" section in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T010 [US1] Write the "From Raw Data to Perception" section, explaining how raw signals are converted to useful information, in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T011 [US1] Write the "An Introduction to Sensor Fusion" section, focusing on the "why" rather than the "how," in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T012 [P] [US1] Create a diagram showing typical sensor placement on a humanoid robot and save it to `textbook/static/img/ch03-sensor-placement.png`.
- [X] T013 [P] [US1] Create a diagram illustrating the data flow from physical sensors to abstract perception modules and save it to `textbook/static/img/ch03-perception-flow.png`.
- [X] T014 [US1] Integrate the diagrams (T012, T013) into the relevant sections of `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T015 [US1] Write the "Summary and Next Steps" section in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.
- [X] T016 [US1] Write the "Practice & Reflection" exercises in `textbook/docs/module1-ros2/03-sensors-and-perception.md`.

**Checkpoint**: At this point, User Story 1 should be fully implemented. The chapter is ready for review.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation that affect the entire chapter.

- [X] T017 Perform a full review of `textbook/docs/module1-ros2/03-sensors-and-perception.md` for clarity, technical accuracy, and grammatical correctness.
- [X] T018 Validate that all content requirements from `specs/004-chapter3-sensors-perception/spec.md` have been met.
- [X] T019 Check for consistent use of terminology with previous chapters and the book's glossary.
- [X] T020 Ensure all diagrams are correctly referenced and clearly explained in the text.

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

- Research tasks (T002, T003) should be completed before writing begins.
- Diagram creation (T012, T013) can happen in parallel with writing tasks but must be completed before the integration task (T014).
- Writing tasks (T005-T016, excluding diagram integration) can largely proceed sequentially.

### Parallel Opportunities

- **T002** and **T003** can be performed in parallel.
- **T012** and **T013** (diagram creation) can be performed in parallel with the writing tasks.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  Complete Phase 4: Polish.
5.  **STOP and VALIDATE**: Review the complete chapter against the spec.
