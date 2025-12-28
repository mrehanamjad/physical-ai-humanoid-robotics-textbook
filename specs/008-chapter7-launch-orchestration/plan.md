# Implementation Plan: Chapter 7 – Launch Files, Parameters, and System Orchestration

**Branch**: `008-chapter7-launch-orchestration` | **Date**: 2025-12-27 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `specs/008-chapter7-launch-orchestration/spec.md`

## Summary

This chapter teaches learners how to manage multi-node ROS 2 systems using Python-based launch files. It covers starting, configuring, and orchestrating multiple nodes, transitioning the learner from a node-level to a system-level mindset. The plan outlines the creation of a system orchestration architecture sketch, a section-by-section writing plan, a research approach for the ROS 2 launch system, and a quality validation checklist.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: ROS 2 Humble, `rclpy`, `launch_ros`
**Storage**: N/A
**Testing**: ROS 2 Launch Testing (`launch_testing`)
**Target Platform**: Ubuntu 22.04 with ROS 2 Humble
**Project Type**: Documentation (Docusaurus Markdown)
**Performance Goals**: Launch files should be efficient and clear. System startup should be robust.
**Constraints**: Content must be Docusaurus-compatible Markdown (MD/MDX). Focus on Python launch files.
**Scale/Scope**: Chapter length of 4,000–5,000 words.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Specification-Driven Development**: This plan is derived from `spec.md`.
- [x] **Technical Accuracy**: The plan is grounded in established ROS 2 principles.
- [x] **Pedagogical Clarity**: The approach is designed for learners progressing from single-node development.
- [x] **AI-Native Authoring**: The plan will leverage AI for drafting and validation.
- [x] **Open Knowledge**: The output will be public and version-controlled.
- [x] **Reproducibility**: All code and steps will be documented and reproducible.

## Project Structure

### Documentation (this feature)

```text
specs/008-chapter7-launch-orchestration/
├── plan.md              # This file
├── research.md          # Phase 0 output
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

The content for this chapter will be a single Markdown file within the Docusaurus site structure.

```text
textbook/
└── docs/
    └── module1-ros2/
        └── 07-launch-files-and-parameters.md
```

**Structure Decision**: A single Markdown file is sufficient as the deliverable is a chapter in the Docusaurus-based textbook.

## Planning Phases

### Phase 0: Research

This phase resolves unknowns related to teaching system orchestration effectively.

1.  **Launch System Internals**:
    -   **Task**: Investigate the relationship between Python launch files, the `launch` library, and the underlying ROS 2 command-line tools.
    -   **Goal**: Decide on the right balance between showing practical launch file examples and explaining the underlying mechanics (Python vs. CLI focus).
2.  **Parameter System Deep Dive**:
    -   **Task**: Analyze the full scope of the ROS 2 parameter system, including parameter types, overrides, and dynamic reconfiguration.
    -   **Goal**: Define the appropriate depth of coverage to provide powerful tools without overwhelming the learner, balancing cognitive load.
3.  **Lifecycle Node Applicability**:
    -   **Task**: Research the states and transitions of lifecycle (managed) nodes.
    -   **Goal**: Determine whether to cover this as a conceptual introduction or a full implementation example, given the chapter's scope.
4.  **Scalable Orchestration Patterns**:
    -   **Task**: Review best practices for structuring launch files in large-scale systems (e.g., real humanoid robots).
    -   **Goal**: Develop examples and diagrams that are relevant to humanoids but simple enough for beginners. Settle the "humanoid-scale vs. generic" example debate.

**Output**: `research.md` containing decisions and rationale for the topics above.

### Phase 1: Foundation (Content Architecture)

This phase maps research to a concrete chapter structure and architectural vision.

1.  **Section-by-Section Writing Plan**:
    -   Align content with the 14 sections defined in `spec.md#FR-002`.
    -   For each section, define learning objectives, key concepts, code examples, and necessary diagrams.
2.  **System Orchestration Architecture Sketch**:
    -   Create a high-level diagram showing a launch file that starts a multi-node system (e.g., a camera driver, an image processor, and a decision-making node).
    -   Illustrate the flow of parameters from a YAML file into specific nodes.
    -   Show how namespaces isolate two instances of the same subsystem.
3.  **Hands-On Exercise Design**:
    -   Design the final exercises to match the acceptance scenarios in `spec.md`, guiding learners to build and configure a complete multi-node system from scratch.

**Output**: An updated `plan.md` with the detailed writing plan and architectural sketches.

### Phase 2: Analysis (Drafting & Validation)

This phase focuses on creating and validating the chapter content.

1.  **Drafting**:
    -   Write the full chapter text in `textbook/docs/module1-ros2/07-launch-files-and-parameters.md`.
    -   Implement all launch files and associated node examples, ensuring they are correct and robust.
    -   Create clear, high-quality diagrams for system orchestration, namespaces, and startup sequences.
2.  **Testing and Validation**:
    -   **Launch Execution**: Run every launch file to guarantee correctness and validate node startup and parameter loading.
    -   **Conceptual Consistency**: Check for alignment with Chapter 6 (`rclpy` nodes) and upcoming chapters (URDF, simulation). The system launched here should be a precursor to the robot models used later.
    -   **Learner Resilience**: Manually test the debugging guide against common, intentionally-injected errors in launch files.

**Output**: The first complete draft of the chapter Markdown file and all associated code/launch files.

### Phase 3: Synthesis (Refinement & Quality Checklist)

This phase polishes the draft and formalizes quality checks.

1.  **Refinement**:
    -   Incorporate feedback from internal reviews, focusing on the clarity of the system-level concepts.
    -   Adjust the balance of diagram density vs. narrative explanation based on review feedback.
    -   Ensure the chapter meets its target word count.
2.  **Quality Validation Checklist**:
    -   **System Correctness**: All launch files behave as described.
    -   **Learner Comprehension**: The chapter effectively builds system-level thinking.
    -   **Completeness**: All requirements from `spec.md` are met.
    -   **Inter-Chapter Alignment**: The concepts and examples flow logically from Chapter 6 and into Chapter 8 and Module 2.

**Output**: The final, polished chapter ready for review and merging.

## Decisions Needing Documentation (ADRs)

-   **ADR-1**: Focus on Python Launch Files vs. CLI-Based Launching.
-   **ADR-2**: Depth of Parameter System Coverage for an Introductory Chapter.
-   **ADR-3**: Scope of Lifecycle Node Discussion (Conceptual vs. Implemented).
-   **ADR-4**: Strategy for Humanoid-Scale vs. Generic Robot Examples.

## Testing Strategy

-   **Launch Testing**: All launch files will be validated using `launch_testing` to automatically verify that all nodes start correctly and parameters are loaded as expected.
-   **Consistency Checks**: A manual review will ensure the nodes launched in this chapter are consistent with the `rclpy` patterns taught in Chapter 6. The system architecture will be checked for compatibility with the URDF models planned for Chapter 8.
-   **Pedagogical Validation**: The hands-on exercises, which require learners to build a multi-node system, will serve as the primary validation of learner readiness for more complex modules.
-   **Diagram Review**: All diagrams will be reviewed for correctness and clarity in depicting the startup sequence and data flow.
