# Implementation Plan: Chapter 6 – Python-Based ROS 2 Development with rclpy

**Branch**: `007-chapter6-rclpy-development` | **Date**: 2025-12-27 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `specs/007-chapter6-rclpy-development/spec.md`

## Summary

This chapter teaches learners how to implement ROS 2 nodes, publishers, subscribers, services, and actions in Python using the `rclpy` library. It bridges conceptual understanding of ROS 2 primitives to practical, hands-on coding, providing the core skills necessary for all subsequent modules. The plan outlines the creation of a node-level software architecture sketch, a section-by-section writing plan, a research approach for `rclpy`, and a quality validation checklist.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: ROS 2 Humble, `rclpy`
**Storage**: N/A
**Testing**: `pytest`, ROS 2 Launch Testing (`launch_testing`)
**Target Platform**: Ubuntu 22.04 with ROS 2 Humble
**Project Type**: Documentation (Docusaurus Markdown)
**Performance Goals**: Code examples must be performant enough for real-time robotics context, but prioritize clarity.
**Constraints**: Content must be Docusaurus-compatible Markdown (MD/MDX). All examples must be self-contained and runnable.
**Scale/Scope**: Chapter length of 4,000–5,000 words.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Specification-Driven Development**: This plan is derived from `spec.md`.
- [x] **Technical Accuracy**: The plan is grounded in established ROS 2 and Python principles.
- [x] **Pedagogical Clarity**: The approach is designed for learners with a CS/engineering background.
- [x] **AI-Native Authoring**: The plan will leverage AI for drafting and validation.
- [x] **Open Knowledge**: The output will be public and version-controlled.
- [x] **Reproducibility**: All code and steps will be documented and reproducible.

## Project Structure

### Documentation (this feature)

```text
specs/007-chapter6-rclpy-development/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (N/A for this chapter)
├── quickstart.md        # Phase 1 output (contains final hands-on exercises)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

The content for this chapter will be a single Markdown file within the Docusaurus site structure.

```text
textbook/
└── docs/
    └── module1-ros2/
        └── 06-python-ros2-development.md
```

**Structure Decision**: A single Markdown file is sufficient as the deliverable is a chapter in the Docusaurus-based textbook. No complex source code structure is needed.

## Planning Phases

### Phase 0: Research

This phase focuses on resolving unknowns and establishing best practices for teaching `rclpy`.

1.  **API Research**:
    -   **Task**: Investigate the core `rclpy` APIs for nodes, publishers, subscribers, services, and actions.
    -   **Goal**: Identify the most straightforward and "beginner-safe" methods and classes to present. Resolve the level of abstraction vs. exposure to ROS 2 internals.
2.  **Execution Model Research**:
    -   **Task**: Analyze single-threaded and multi-threaded executors in `rclpy`.
    -   **Goal**: Develop a clear, visualizable explanation for the callback-based execution model ("spinning") to prevent learner confusion.
3.  **Best Practices Research**:
    -   **Task**: Research common `rclpy` patterns and anti-patterns in open-source robotics projects.
    -   **Goal**: Formulate a "style guide" for chapter examples, covering naming conventions, class structure, and error handling. Define the scope of examples (simple nodes vs. small coordinated systems).
4.  **Performance Research**:
    -   **Task**: Research the performance implications of using Python for ROS 2 nodes.
    -   **Goal**: Document key tradeoffs to present to learners regarding Python's ease of use versus C++'s performance for high-frequency tasks.

**Output**: `research.md` containing decisions, rationales, and alternatives for the topics above.

### Phase 1: Foundation (Content Architecture)

This phase maps the research findings to a concrete chapter structure and a high-level architectural sketch.

1.  **Section-by-Section Writing Plan**:
    -   Align content with the 13 sections defined in `spec.md#FR-002`.
    -   For each section, define:
        -   **Learning Objective**: What will the learner be able to do?
        -   **Key Concepts**: What terms and ideas will be introduced?
        -   **Code Examples**: What specific, minimal code will be shown?
        -   **Diagrams**: What visual aids are needed (e.g., node lifecycle, callback flow)?
2.  **Node-Level Software Architecture Sketch**:
    -   Create a conceptual diagram showing a typical `rclpy` system.
    -   Illustrate 2-3 nodes interacting via topics and services.
    -   This sketch will serve as a visual anchor for the entire chapter.
3.  **Hands-On Exercise Design**:
    -   Design the final `quickstart.md` exercises based on the acceptance scenarios in `spec.md`.
    -   The exercises will guide learners to build a simple, multi-node system (e.g., a "greeter" bot).

**Output**: An updated `plan.md` with the detailed writing plan. A preliminary `quickstart.md`.

### Phase 2: Analysis (Drafting & Validation)

This phase involves creating the chapter content and ensuring its quality.

1.  **Drafting**:
    -   Write the full chapter text in `textbook/docs/module1-ros2/06-python-ros2-development.md`.
    -   Implement all code examples, ensuring they are runnable and follow the style guide from Phase 0.
    -   Create diagrams using a consistent, clear style.
2.  **Testing and Validation**:
    -   **Code Execution**: Run every code snippet and exercise to guarantee correctness.
    -   **Conceptual Consistency**: Cross-reference with Chapter 5 (Primitives) and Chapter 7 (Launch) to ensure a seamless learning path.
    -   **Learner Comprehension**: Internally review the chapter from a beginner's perspective. Does it address the "Callback Confusion" edge case? Is the debugging section helpful?

**Output**: The first complete draft of the chapter Markdown file and all associated code.

### Phase 3: Synthesis (Refinement & Quality Checklist)

This phase polishes the draft and formalizes the quality checks.

1.  **Refinement**:
    -   Incorporate feedback from the internal review.
    -   Refine language for clarity, conciseness, and pedagogical effectiveness.
    -   Check for compliance with the 4,000-5,000 word count target.
2.  **Quality Validation Checklist**:
    -   **Code Correctness**: All examples run without errors.
    -   **Clarity**: Explanations are easy to understand for the target audience.
    -   **Learner Readiness**: The chapter successfully prepares learners for subsequent modules.
    -   **Completeness**: All requirements from `spec.md` are met.
    -   **Consistency**: Terminology and concepts align with the rest of the textbook.
3.  **ADR Documentation**:
    -   Create ADRs for the key decisions identified in the prompt.

**Output**: The final, polished chapter ready for review and merging.

## Decisions Needing Documentation (ADRs)

-   **ADR-1**: Level of Python Abstraction vs. ROS 2 Internals Exposure.
-   **ADR-2**: Scope of Examples (Simple Nodes vs. Coordinated Systems).
-   **ADR-3**: Explanation Strategy for Callback-Based Execution.
-   **ADR-4**: Depth of Actions Implementation (Introductory vs. Advanced).

## Testing Strategy

-   **Unit Testing**: Individual `rclpy` code snippets will be tested for correctness via `pytest`.
-   **Integration Testing**: The hands-on exercises, which involve multiple nodes, will be tested using ROS 2 launch testing (`launch_testing`) to validate inter-node communication.
-   **Conceptual Validation**: A manual review will check for consistency with Chapters 5 and 7.
-   **Pedagogical Validation**: The hands-on exercises serve as the primary test for learner comprehension, mirroring the acceptance scenarios in the spec.
-   **Linting/Style**: All Python code will be checked against a standard linter (e.g., `flake8` or `black`).
