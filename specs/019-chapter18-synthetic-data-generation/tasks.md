# Tasks: Chapter 18 – Synthetic Data Generation

**Input**: Design documents from `/specs/019-chapter18-synthetic-data-generation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. This chapter is heavily focused on a hands-on tutorial.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

The chapter file will be created at `textbook/docs/module3/chapter18-synthetic-data-generation.mdx`. Supporting code and scripts will be in `textbook/code/chapter18/`.

## Phase 1: Setup (Shared Infrastructure)

- [ ] T001 Create the chapter file `textbook/docs/module3/chapter18-synthetic-data-generation.mdx`.
- [ ] T002 Create the directory `textbook/code/chapter18/` for Python scripts and examples.

---

## Phase 2: User Story 1 - Understanding the Value of Synthetic Data (Priority: P1)

**Goal**: A learner understands why generating data in simulation is a transformative approach for training robot AI systems.

**Independent Test**: The learner can explain the top three advantages of synthetic data over real-world data collection.

### Implementation for User Story 1

- [ ] T003 [US1] Write the "Chapter Overview" section in the chapter file, highlighting the shift from testing to data generation.
- [ ] T004 [US1] Write the "Introduction to Synthetic Data" section, defining core concepts and explaining the benefits (cost, scale, automatic annotation).
- [ ] T005 [US1] Create and embed a diagram of the "Synthetic Data Generation Pipeline" (Scene Setup → Annotation → Export).

**Checkpoint**: User Story 1 is complete. The conceptual foundation for the chapter is established.

---

## Phase 3: User Story 2 - Generating and Annotating a Dataset (Priority: P2) 🎯 MVP

**Goal**: A learner can set up a scene in Isaac Sim, record data, and generate annotated images.

**Independent Test**: The learner can produce a small dataset of RGB images and corresponding segmentation masks.

### Implementation for User Story 2

- [ ] T006 [US2] Write the "Hands-On: Your First Dataset" tutorial section in the chapter file.
- [ ] T007 [US2] Sub-task: Provide a step-by-step guide for setting up a simple scene in Isaac Sim optimized for data generation (e.g., with multiple objects of interest).
- [ ] T008 [US2] Sub-task: Write instructions on how to use the Replicator/Annotator UI or a basic script to capture RGB images and semantic segmentation masks.
- [ ] T009 [US2] Sub-task: Detail the process for exporting the captured data to the local filesystem.
- [ ] T010 [US2] Write the "Consuming the Data" section, providing a simple Python script in `textbook/code/chapter18/verify_dataset.py` to load and visualize the generated images and masks.

**Checkpoint**: User Story 2 is complete. The learner has successfully generated and verified their first synthetic dataset.

---

## Phase 4: User Story 3 - Applying Domain Randomization (Priority: P3)

**Goal**: A learner can apply domain randomization to a scene to increase the diversity of the generated dataset.

**Independent Test**: The learner can configure a randomization script that produces visibly different images on each run.

### Implementation for User Story 3

- [ ] T011 [US3] Write the "Bridging the Sim-to-Real Gap" section in the chapter file.
- [ ] T012 [US3] Sub-task: Explain the concept of Domain Randomization (DR) and why it helps models generalize.
- [ ] T013 [US3] Sub-task: Provide a hands-on guide for applying DR in Isaac Sim, showing how to randomize lighting conditions and object textures.
- [ ] T014 [US3] Include example GIFs or images showing the output of a scene with and without DR enabled.

**Checkpoint**: User Story 3 is complete. The learner can apply a key technique for improving sim-to-real transfer.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the chapter with supporting materials and perform validation.

- [ ] T015 Write the "Summary & Next Steps" section, summarizing the workflow and connecting it to future AI training tasks.
- [ ] T016 **(CRITICAL)** Perform a full end-to-end test of the tutorial, from scene setup to data verification, on a clean Isaac Sim installation.
- [ ] T017 Review the entire chapter for technical accuracy, clarity of instructions, and consistency with Chapter 17.

---

## Dependencies & Execution Order

- **Setup (Phase 1)**: Must be completed first.
- **User Stories (Phase 2-4)**: Must be completed in order (US1 → US2 → US3) as they represent a continuous, building workflow.
- **Polish (Phase 5)**: Depends on all user stories being complete, with a special focus on the end-to-end validation test.

## Implementation Strategy

The implementation will follow the task order precisely to create a successful hands-on tutorial.

1.  Complete Phase 1 (Setup).
2.  Complete Phase 2 (US1) to provide conceptual context.
3.  Complete Phase 3 (US2) for the core hands-on data generation workflow.
4.  Complete Phase 4 (US3) to add the crucial domain randomization technique.
5.  Complete Phase 5 (Polish) to ensure the tutorial is robust and reproducible.
