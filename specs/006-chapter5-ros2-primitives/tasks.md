# Tasks: Chapter 5 – ROS 2 Communication Primitives

**Input**: Design documents from `/specs/006-chapter5-ros2-primitives/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1)

## Path Conventions

- All paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the primary file for the chapter.

- [X] T001 Create the chapter markdown file in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Complete the necessary research before writing begins.

- [X] T002 [P] Research precise definitions and effective analogies for Topics, Services, and Actions from official ROS 2 documentation and community resources.
- [X] T003 [P] Identify clear, canonical robotic use cases for each communication primitive.
- [X] T004 Consolidate research findings, including the pedagogical order of introduction, into `specs/006-chapter5-ros2-primitives/research.md`.

**Checkpoint**: Foundation ready - chapter writing can now begin.

---

## Phase 3: User Story 1 - Building a Mental Model of ROS 2 Communication (Priority: P1) 🎯 MVP

**Goal**: To write a chapter that builds a strong mental model of ROS 2's core communication primitives (Nodes, Topics, Services, Actions) and the criteria for choosing between them.

**Independent Test**: A reader, after completing the chapter, can correctly design a ROS 2 graph for a simple robotics task, justifying their choice of communication primitives.

### Implementation for User Story 1

- [X] T005 [US1] Write the "Chapter Overview" section in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T006 [US1] Write the "Recap: The ROS 2 Computational Graph" section in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T007 [US1] Write the "Topics: The Continuous Data Stream" section, including an analogy and use cases, in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T008 [P] [US1] Create a data flow diagram for a Topic and save it to `textbook/static/img/ch05-topic-flow.png`.
- [X] T009 [US1] Write the "Services: The Request-Response Interaction" section, including an analogy and use cases, in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T010 [P] [US1] Create a data flow diagram for a Service and save it to `textbook/static/img/ch05-service-flow.png`.
- [X] T011 [US1] Write the "Actions: For Long-Running Goals with Feedback" section, including an analogy and use cases, in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T012 [P] [US1] Create a data flow diagram for an Action, showing the full lifecycle, and save it to `textbook/static/img/ch05-action-flow.png`.
- [X] T013 [US1] Write the "Choosing the Right Tool" section, including a comparative table, in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T014 [US1] Write the "Conceptual Executors: Managing the Flow" section in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T015 [US1] Integrate all diagrams (T008, T010, T012) into their relevant sections.
- [X] T016 [US1] Write the "Summary and Next Steps" section in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.
- [X] T017 [US1] Write the "Conceptual Exercises & Design Scenarios" section in `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md`.

**Checkpoint**: At this point, User Story 1 should be fully implemented. The chapter is ready for review.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and validation that affect the entire chapter.

- [X] T018 Perform a full review of `textbook/docs/module1-ros2/05-ros2-nodes-topics-services-actions.md` for clarity, technical accuracy, and grammatical correctness.
- [X] T019 Validate that all content requirements from `specs/006-chapter5-ros2-primitives/spec.md` have been met.
- [X] T020 Check for consistent use of terminology with previous chapters and the book's glossary.
- [X] T021 Ensure all diagrams are correctly referenced and clearly explained in the text.

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
- Diagram creation (T008, T010, T012) can happen in parallel with writing tasks but must be completed before the integration task (T015).
- Writing tasks can largely proceed sequentially as they build a narrative.

### Parallel Opportunities

- **T002** and **T003** can be performed in parallel.
- **T008**, **T010**, and **T012** (diagram creation) can be performed in parallel with the writing tasks.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  Complete Phase 4: Polish.
5.  **STOP and VALIDATE**: Review the complete chapter against the spec.
