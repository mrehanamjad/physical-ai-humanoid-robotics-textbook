# Tasks: High-Level Book Layout

**Input**: Design documents from `/specs/001-book-layout/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the Docusaurus project and configure the basic structure.

- [x] T001 Initialize a new Docusaurus site using the `npx create-docusaurus@latest` command.
- [x] T002 Configure `textbook/docusaurus.config.js` with the book title, subtitle, and navigation structure.
- [x] T003 [P] Configure `textbook/sidebars.js` to manage the sidebar for the book's modules and chapters.
- [x] T004 [P] Customize the homepage by editing `textbook/src/pages/index.js` to reflect the book's content.

---

## Phase 2: Foundational Content

**Purpose**: Create the main landing page for the documentation section.

- [x] T005 Create the main documentation landing page at `textbook/docs/intro.md`.

---

## Phase 3: User Story 1 - Create Module 1 (ROS 2) (Priority: P1) 🎯 MVP

**Goal**: Create the file structure and placeholder content for the "Robotic Nervous System (ROS 2)" module.

### Implementation for User Story 1
- [x] T006 [US1] Create the directory for the ROS 2 module at `textbook/docs/module1-ros2`.
- [x] T007 [US1] Create the category definition file `textbook/docs/module1-ros2/_category_.json` to set the module title.
- [x] T008 [P] [US1] Create Chapter 1 placeholder file: `textbook/docs/module1-ros2/01-introduction-to-ros2.md`.
- [x] T009 [P] [US1] Create Chapter 2 placeholder file: `textbook/docs/module1-ros2/02-ros2-nodes.md`.
- [x] T010 [P] [US1] Create Chapter 3 placeholder file: `textbook/docs/module1-ros2/03-ros2-topics.md`.

**Checkpoint**: At this point, the ROS 2 module should appear in the sidebar with its chapters, and the pages should be accessible.

---

## Phase 4: User Story 2 - Create Module 2 (Digital Twin) (Priority: P2)

**Goal**: Create the file structure and placeholder content for the "Digital Twin (Gazebo & Unity)" module.

### Implementation for User Story 2
- [x] T011 [US2] Create the directory for the Digital Twin module at `textbook/docs/module2-gazebo`.
- [x] T012 [US2] Create the category definition file `textbook/docs/module2-gazebo/_category_.json` to set the module title.
- [x] T013 [P] [US2] Create Chapter 1 placeholder file: `textbook/docs/module2-gazebo/01-introduction-to-simulation.md`.
- [x] T014 [P] [US2] Create Chapter 2 placeholder file: `textbook/docs/module2-gazebo/02-building-worlds-in-gazebo.md`.

**Checkpoint**: The Digital Twin module should appear in the sidebar with its chapters.

---

## Phase 5: User Story 3 - Create Module 3 (AI-Robot Brain) (Priority: P3)

**Goal**: Create the file structure and placeholder content for the "AI-Robot Brain (NVIDIA Isaac)" module.

### Implementation for User Story 3
- [x] T015 [US3] Create the directory for the AI-Robot Brain module at `textbook/docs/module3-isaac`.
- [x] T016 [US3] Create the category definition file `textbook/docs/module3-isaac/_category_.json` to set the module title.
- [x] T017 [P] [US3] Create Chapter 1 placeholder file: `textbook/docs/module3-isaac/01-introduction-to-isaac-sim.md`.
- [x] T018 [P] [US3] Create Chapter 2 placeholder file: `textbook/docs/module3-isaac/02-isaac-sim-ros2-integration.md`.

**Checkpoint**: The AI-Robot Brain module should appear in the sidebar with its chapters.

---

## Phase 6: User Story 4 - Create Module 4 (VLA) (Priority: P4)

**Goal**: Create the file structure and placeholder content for the "Vision-Language-Action (VLA)" module.

### Implementation for User Story 4
- [x] T019 [US4] Create the directory for the VLA module at `textbook/docs/module4-vla`.
- [x] T020 [US4] Create the category definition file `textbook/docs/module4-vla/_category_.json` to set the module title.
- [x] T021 [P] [US4] Create Chapter 1 placeholder file: `textbook/docs/module4-vla/01-introduction-to-vla-models.md`.

**Checkpoint**: The VLA module should appear in the sidebar with its chapter.

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Ensure the overall quality and consistency of the book structure.

- [x] T022 Review all generated markdown files for correct front-matter and structure.
- [x] T023 Run a local build of the Docusaurus site (`npm run build`) in the `textbook` directory to ensure there are no errors.
- [x] T024 Manually check all sidebar links to confirm they navigate to the correct pages.

---

## Dependencies

The user stories can be implemented in parallel after the Setup and Foundational phases are complete, but a sequential order is recommended for clarity.

-   **Phase 1 & 2** are prerequisites for all other phases.
-   **Phase 3 (US1)** -> **Phase 4 (US2)** -> **Phase 5 (US3)** -> **Phase 6 (US4)** is the recommended logical flow, but they are not strictly dependent on each other for file creation.

## Parallel Execution

-   Within each user story phase, the creation of chapter files (e.g., `T008`, `T009`, `T010`) can be done in parallel as they are independent of each other.

## Implementation Strategy

The project will be delivered incrementally, with each user story representing a deliverable module structure. The MVP (Minimum Viable Product) is the completion of User Story 1, which establishes the structure for the first module of the book.
