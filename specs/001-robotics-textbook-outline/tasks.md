# Actionable Tasks for: Book Content Outline

**Implementation Plan**: [plan.md](./plan.md)
**Feature Spec**: [spec.md](./spec.md)

---

## Phase 1: Setup

- [X] T001 Initialize Docusaurus project in the `textbook/` directory
- [X] T002 Configure `docusaurus.config.ts` with the textbook title, theme, and other basic settings
- [X] T003 [P] Create a basic `sidebars.ts` file in the `textbook/` directory

## Phase 2: Foundational Tasks

- [X] T004 Create the directory structure for the four modules in `textbook/docs/`: `module1-ros2`, `module2-gazebo`, `module3-isaac`, `module4-vla`
- [X] T005 [P] Create a placeholder `_category_.json` for each module directory to define its label and position
- [X] T006 [P] Create an introductory `intro.md` file in `textbook/docs/` to provide an overview of the textbook

## Phase 3: User Story 1 - Curriculum Designer Defines Book Structure

**Goal**: Define a comprehensive and logically structured table of contents.
**Independent Test**: The generated outline can be reviewed and validated against the specified module and chapter requirements.

- [X] T007 [US1] Create chapter files for Module 1 in `textbook/docs/module1-ros2/`
- [X] T008 [P] [US1] Create `01-physical-ai-foundations.md`
- [X] T009 [P] [US1] Create `02-humanoid-robotics-overview.md`
- [X] T010 [P] [US1] Create `03-sensors-and-perception.md`
- [X] T011 [P] [US1] Create `04-ros2-architecture.md`
- [X] T012 [P] [US1] Create `05-ros2-nodes-topics-services-actions.md`
- [X] T013 [P] [US1] Create `06-python-ros2-development.md`
- [X] T014 [P] [US1] Create `07-launch-files-and-parameters.md`
- [X] T015 [P] [US1] Create `08-urdf-for-humanoid-modeling.md`

- [X] T016 [US1] Create chapter files for Module 2 in `textbook/docs/module2-gazebo/`
- [X] T017 [P] [US1] Create `01-digital-twins-and-simulation.md`
- [X] T018 [P] [US1] Create `02-gazebo-environment-setup.md`
- [X] T019 [P] [US1] Create `03-physics-simulation.md`
- [X] T020 [P] [US1] Create `04-robot-description-formats-sdf-vs-urdf.md`
- [X] T021 [P] [US1] Create `05-sensor-simulation.md`
- [X] T022 [P] [US1] Create `06-unity-for-visualization.md`
- [X] T023 [P] [US1] Create `07-simulation-validation.md`

- [X] T024 [US1] Create chapter files for Module 3 in `textbook/docs/module3-isaac/`
- [X] T025 [P] [US1] Create `01-nvidia-isaac-ecosystem-overview.md`
- [X] T026 [P] [US1] Create `02-isaac-sim-and-photorealistic-simulation.md`
- [X] T027 [P] [US1] Create `03-synthetic-data-generation.md`
- [X] T028 [P] [US1] Create `04-isaac-ros-and-hardware-acceleration.md`
- [X] T029 [P] [US1] Create `05-visual-slam.md`
- [X] T030 [P] [US1] Create `06-navigation-with-nav2.md`
- [X] T031 [P] [US1] Create `07-reinforcement-learning-for-robot-control.md`
- [X] T032 [P] [US1] Create `08-sim-to-real-transfer.md`

- [X] T033 [US1] Create chapter files for Module 4 in `textbook/docs/module4-vla/`
- [X] T034 [P] [US1] Create `01-humanoid-robot-kinematics-and-dynamics.md`
- [X] T035 [P] [US1] Create `02-bipedal-locomotion-and-balance.md`
- [X] T036 [P] [US1] Create `03-manipulation-and-grasping.md`
- [X] T037 [P] [US1] Create `04-natural-human-robot-interaction.md`
- [X] T038 [P] [US1] Create `05-vision-language-action-paradigms.md`
- [X] T039 [P] [US1] Create `06-voice-based-control.md`
- [X] T040 [P] [US1] Create `07-cognitive-planning.md`
- [X] T041 [P] [US1] Create `08-capstone-project.md`

- [X] T042 [US1] Configure the sidebar in `sidebars.ts` to reflect the module and chapter structure

## Phase 4: User Story 2 - Student Previews Learning Journey

**Goal**: Allow students to understand the scope and progression of the course.
**Independent Test**: A student can read the generated table of contents and get a clear understanding of what they will learn.

- [X] T043 [US2] Review the generated Docusaurus site to ensure the sidebar navigation is clear and intuitive
- [X] T044 [US2] Validate that all chapter titles accurately reflect their content and learning objectives
- [X] T045 [P] [US2] Add a brief description to the front matter of each chapter's Markdown file

## Phase 5: Polish & Cross-Cutting Concerns

- [X] T046 Review all generated files for consistency and correctness
- [X] T047 [P] Ensure all internal links between chapters and modules are working correctly
- [X] T048 [P] Add placeholder content to each chapter file, including the standard section structure (e.g., Overview, Learning Objectives, etc.)

---

## Dependencies

- **US2 depends on US1**: The learning journey cannot be previewed until the book structure is defined.

## Parallel Execution

- Within each module's chapter creation tasks (e.g., T008-T015), the tasks can be executed in parallel as they are independent file creations.
- Tasks in Phase 4 and 5 can be parallelized.

## Implementation Strategy

The implementation will follow a phased approach, starting with the setup of the Docusaurus project and the creation of the foundational module structure. User Story 1 will be implemented first, as it is the core of the feature. User Story 2 will then be addressed to ensure the content is presented in a clear and understandable manner. The final phase will focus on polishing the content and ensuring consistency across the textbook.
