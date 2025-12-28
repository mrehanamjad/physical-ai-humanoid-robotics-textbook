# Tasks: Chapter 4 – ROS 2 Architecture and Core Concepts

**Input**: Design documents from `/specs/005-chapter4-ros2-architecture/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)

## Path Conventions

- All paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the primary file for the chapter.

- [X] T001 Create the chapter markdown file in `textbook/docs/module1-ros2/04-ros2-architecture.md`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Complete the necessary research before writing begins.

- [X] T002 [P] Thoroughly read the official ROS 2 documentation's "Concepts" and "Design" sections from `docs.ros.org`.
- [X] T003 [P] Find and review high-quality community explanations of the ROS 2 architecture for beginners.
- [X] T004 Consolidate research findings, including strong analogies for abstract concepts, into `specs/005-chapter4-ros2-architecture/research.md`.

**Checkpoint**: Foundation ready - chapter writing can now begin.

---

## Phase 3: User Story 1 - Understanding the Robotic Nervous System (Priority: P1) 🎯 MVP

**Goal**: To write a chapter that builds a strong mental model of the ROS 2 architecture, its design philosophy, and its core communication concepts.

**Independent Test**: A reader, after completing the chapter, can accurately sketch the ROS 2 layered architecture and map a simple robotics task to a ROS 2 computational graph.

### Implementation for User Story 1

- [X] T005 [US1] Write the "Chapter Overview" section, introducing ROS 2 as the robot's nervous system, in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T006 [US1] Write the "Why ROS 2? The Evolution from ROS 1" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T007 [US1] Write "The ROS 2 Design Philosophy" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T008 [US1] Write "The Layered Architecture (The Stack)" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T009 [P] [US1] Create the ROS 2 layered architecture diagram and save it to `textbook/static/img/ch04-ros2-stack.png`.
- [X] T010 [US1] Integrate the stack diagram (T009) into the "Layered Architecture" section.
- [X] T011 [US1] Write the "DDS: The Industrial-Grade Backbone" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T012 [US1] Write "The Computational Graph: Nodes" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T013 [US1] Write "The Computational Graph: Communication" section, defining Topics, Services, and Actions, in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T014 [US1] Write the "Putting It Together: A Graph Example" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T015 [P] [US1] Create the example ROS 2 computational graph diagram and save it to `textbook/static/img/ch04-computation-graph.png`.
- [X] T016 [US1] Integrate the graph diagram (T015) into the "Graph Example" section.
- [X] T017 [US1] Write "The Execution Model: Processes and Executors" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T018 [US1] Write the "Summary and Next Steps" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.
- [X] T019 [US1] Write the "Conceptual Exercises & Reflection" section in `textbook/docs/module1-ros2/04-ros2-architecture.md`.

**Checkpoint**: At this point, User Story 1 should be fully implemented. The chapter is ready for review.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation that affect the entire chapter.

- [X] T020 Perform a full review of `textbook/docs/module1-ros2/04-ros2-architecture.md` for clarity, technical accuracy, and grammatical correctness.
- [X] T021 Validate that all content requirements from `specs/005-chapter4-ros2-architecture/spec.md` have been met.
- [X] T022 Check for consistent use of terminology with previous chapters and the book's glossary.
- [X] T023 Ensure all diagrams are correctly referenced and clearly explained in the text.

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
- Diagram creation (T009, T015) can happen in parallel with writing tasks but must be completed before their respective integration tasks (T010, T016).
- Writing tasks can largely proceed sequentially as they build a narrative.

### Parallel Opportunities

- **T002** and **T003** can be performed in parallel.
- **T009** and **T015** (diagram creation) can be performed in parallel with the writing tasks.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  Complete Phase 4: Polish.
5.  **STOP and VALIDATE**: Review the complete chapter against the spec.
