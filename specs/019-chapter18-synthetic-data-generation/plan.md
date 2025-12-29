# Architectural Plan: Chapter 18 – Synthetic Data Generation

**Feature Branch**: `019-chapter18-synthetic-data-generation`
**Version**: 1.0
**Status**: DRAFT

---

## 1. Scope and Dependencies

### 1.1. In Scope
- **Conceptual Explanation**: Detailing the *why* and *how* of synthetic data generation for robotics.
- **Technical Workflow**: A step-by-step guide to generating annotated data using Isaac Sim, covering scene setup, sensor simulation, data annotation (semantic/instance segmentation, depth, etc.), and export.
- **Domain Randomization**: Practical explanation and implementation guide for randomizing environmental parameters (lighting, textures, object placement) to improve model robustness.
- **Data Formats**: Overview of common output formats and their integration with AI training frameworks like PyTorch.
- **Hands-On Tutorial**: A complete, follow-along exercise that allows a learner to produce a small, annotated dataset.

### 1.2. Out of Scope
- **Advanced Simulation Techniques**: Deep dives into physics-based rendering, ray tracing internals, or complex material graphs.
- **AI Model Training**: This chapter focuses on *data generation*. While it will show how the data is *prepared* for training, it will not cover the implementation of ML models themselves.
- **Real-World Data**: The chapter will not cover techniques for collecting, cleaning, or labeling real-world datasets. The focus is exclusively synthetic.
- **Alternative Simulators**: The implementation will be standardized on **NVIDIA Isaac Sim**, as per the project constitution and the preceding chapter. Gazebo or Unity will only be mentioned for context if necessary.

### 1.3. External Dependencies
- **NVIDIA Isaac Sim**: Requires a working installation. The plan assumes learners have completed the setup from Chapter 17.
- **Docusaurus**: All content must be in Docusaurus-compatible Markdown (MDX).
- **Project Constitution**: Adherence to all technical and pedagogical standards outlined in `.specify/memory/constitution.md`.

---

## 2. Key Decisions and Rationale (ADRs)

### ADR-001: Balance of Conceptual vs. Implementation Detail
- **Decision**: The chapter will follow a **70/30 rule**: 70% hands-on implementation and 30% conceptual theory. The primary goal is to empower learners with practical skills. Theory will be introduced to justify *why* a step is necessary (e.g., "Why we use domain randomization").
- **Rationale**: The target audience (engineers and CS students) learns best by doing. Pure theory without application is less impactful for this domain. This ratio aligns with the user story focused on a hands-on tutorial.
- **Alternatives**: A 50/50 split was considered but rejected to keep the chapter focused and prevent it from becoming too academic and less of a practical guide.

### ADR-002: Coverage of Photorealism vs. Variability
- **Decision**: Prioritize **variability over photorealism**. The chapter will emphasize techniques like domain randomization that generate a wide diversity of data, even if individual images are not perfectly photorealistic.
- **Rationale**: For training robust perception models, the diversity of data (different lighting, textures, positions) is often more critical than achieving cinematic-quality rendering. This is key for successful sim-to-real transfer.
- **Alternatives**: Focusing on photorealism would require deep dives into advanced rendering settings, which is out of scope and less critical for the learning objective.

### ADR-003: Inclusion of Visual Examples
- **Decision**: The chapter will be heavily illustrated with **images, diagrams, and short videos/GIFs**.
- **Rationale**: Visual concepts are best explained visually. A diagram of the data pipeline, example segmentation masks, and GIFs of domain randomization are non-negotiable for clarity. This aligns with the "Pedagogical Clarity" principle.

---

## 3. Interfaces and Workflow

### 3.1. Synthetic Data Generation Workflow
A central diagram and section will illustrate the following pipeline:
1.  **Scene Setup**: Place robot, objects, and define the environment in Isaac Sim.
2.  **Sensor Configuration**: Attach and configure a virtual camera to the robot.
3.  **Annotation Configuration**: Enable annotators (e.g., `RtxSensorCpu` for segmentation).
4.  **Orchestration**: Use a script (or the UI) to run the simulation, move the camera/robot, and trigger data capture.
5.  **Data Export**: Save the generated data (RGB images, segmentation masks, etc.) to disk in a structured format.
6.  **Verification**: A simple script will be provided to show how to load and visualize the exported dataset.

![Workflow Sketch](https://i.imgur.com/example.png "A placeholder for the data generation pipeline diagram")
*(Note: A formal diagram will be created for the textbook.)*

### 3.2. Section-by-Section Writing Plan

| Section                               | Content                                                                                                                                                                                            | FR-* |
| -------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |------|
| **1. Chapter Overview**                | - Motivation: Why synthetic data is a game-changer for robotics.<br>- Benefits: Speed, safety, scale, automatic labeling.<br>- Limitations: The "sim-to-real gap."                                     | FR-001 |
| **2. Introduction to Synthetic Data** | - Definition and core types (images, depth, segmentation).<br>- The full data generation pipeline (diagram).                                                                                        | FR-002 |
| **3. Hands-On: Your First Dataset**    | - **Tutorial**: Step-by-step guide in Isaac Sim.<br>- Scene setup, adding a camera, running the replicator/annotator.<br>- Exporting RGB images and segmentation masks.                                | FR-004, FR-007 |
| **4. Bridging the Sim-to-Real Gap**    | - Introduction to the reality gap.<br>- **Domain Randomization (DR)**: Definition and importance.<br>- Practical DR: Randomizing textures, lighting, and object positions in Isaac Sim.          | FR-005 |
| **5. Consuming the Data**              | - Overview of common data formats (e.g., KITTI, COCO).<br>- Example Python script to load and display the generated data.<br>- Connecting the dots to model training.                            | FR-006 |
| **6. Summary & Next Steps**            | - Recap of key learnings.<br>- How this skill connects to the next chapters on perception and navigation.                                                                                           | FR-008 |

---

## 4. Research Approach

- **Tooling**: All implementation will be validated against the latest stable release of **NVIDIA Isaac Sim**.
- **Best Practices**: Research will be conducted on current best practices for sim-to-real transfer, focusing on papers and tutorials from NVIDIA, Google DeepMind, and academic robotics labs.
- **Concurrent Writing**: Research and writing will occur concurrently. As a technical concept is explored (e.g., a specific annotator in Isaac Sim), the corresponding section will be drafted.

---

## 5. Quality Validation Checklist

- **[ ] Technical Accuracy**: Is every instruction and explanation related to Isaac Sim correct and reproducible?
- **[ ] Conceptual Clarity**: Does the chapter clearly explain *why* each step is taken?
- **[ ] Learner-Centricity**: Can a learner with the stated prerequisites follow the tutorial and succeed? (User Story 2)
- **[ ] Completeness**: Does the chapter cover all functional requirements from the spec?
- **[ ] Diagram Quality**: Are all diagrams, images, and GIFs clear and directly supportive of the text? (FR-009)
- **[ ] Consistency**: Is the terminology consistent with previous and subsequent chapters?

---

## 6. Testing Strategy

- **Conceptual Checks**: Review against `spec.md` to ensure all learning objectives are met.
- **Implementation Testing**: The hands-on tutorial will be executed end-to-end by a separate reviewer to ensure it is bug-free and the instructions are unambiguous.
- **Scenario Validation**: Test against the user stories. Does the final chapter empower a learner to succeed in those scenarios?
- **Peer Review**: The draft will be reviewed by at least one other person familiar with Isaac Sim to catch technical errors or unclear explanations.
- **Diagram Review**: All visual aids will be checked for accuracy and clarity.
