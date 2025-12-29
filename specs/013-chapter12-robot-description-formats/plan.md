# Implementation Plan: Chapter 12 – Robot Description Formats (SDF vs. URDF)

**Feature Branch**: `013-chapter12-robot-description-formats`
**Implementation Plan**: `specs/013-chapter12-robot-description-formats/plan.md`
**Feature Spec**: `specs/013-chapter12-robot-description-formats/spec.md`

## 1. Technical Context & Design Philosophy

This chapter addresses a fundamental and often confusing topic for robotics students: the existence of two similar-but-different file formats for describing robots. The core philosophy is **"Clarify the 'Why' Behind the 'What'."** Instead of just presenting two standards, the chapter must explain *why* they both exist, their different design goals, and their complementary roles in the ROS/Gazebo ecosystem.

The plan is to introduce URDF first, as it is the simpler, robot-centric format that learners are already familiar with from Chapter 8. SDF will then be introduced as a more powerful, simulation-focused superset of URDF's capabilities. The key takeaway for the learner should be a clear mental model: **URDF describes the robot's geometry for kinematics and visualization; SDF describes the entire world for simulation.**

## 2. Constitution Check

- [X] **Specification-Driven Development**: This plan is directly derived from `spec.md`, focusing on explaining the two formats and their interplay.
- [X] **Technical Correctness**: Research is required to ensure the feature comparison and the description of the URDF-to-SDF conversion process are accurate.
- [X] **Pedagogical Clarity**: The "URDF-first, then SDF" approach provides a progressive learning path, building on prior knowledge.
- [X] **AI-Native Authoring**: AI will be used to draft explanations and generate descriptions for the comparison diagrams.
- [X] **Open Knowledge**: The focus is on open-source formats used in the ROS and Gazebo ecosystems.
- [X] **Reproducibility & Traceability**: The testing strategy ensures that the conceptual models presented are consistent with the behavior of the actual software tools.

**Gate Evaluation**: Pass. All principles are met.

## 3. Implementation Phases

### Phase 0: Research & Foundation

**Objective**: Solidify the conceptual framework and decide on the pedagogical approach for comparing the two formats.

**Tasks**:

1.  **Research Format Specifics**:
    *   Task: "Analyze the key structural differences between a typical URDF file and a corresponding SDF file for the same robot."
    *   Task: "Investigate the current state of the automatic URDF-to-SDF conversion performed by Gazebo. What are the common failure points or information losses (e.g., missing `<gazebo>` tags)?"
2.  **Architectural Sketches**:
    *   Task: "Draft a description for a diagram illustrating the **Robot Description Pipeline**." It must show:
        *   A box for "humanoid.urdf".
        *   An arrow to a box for "ROS 2 `robot_state_publisher`", with an annotation "Publishes `/tf` for visualization in RViz."
        *   Another arrow from the URDF to a box for "Gazebo `spawn_entity` service."
        *   Inside Gazebo, show a box labeled "Internal URDF-to-SDF Converter."
        *   The converter points to the final "Simulated Robot (SDF)" inside the Gazebo environment.
    *   Task: "Draft a description for a **Side-by-Side Comparison Diagram**." This will be a two-column table or diagram:
        *   **Left Column (URDF)**: Labeled "Robot-Centric". Shows a single `<robot>` tag containing `<link>` and `<joint>` elements.
        *   **Right Column (SDF)**: Labeled "World-Centric". Shows a `<world>` tag that contains `<model>`, `<light>`, and `<physics>`. The `<model>` tag then contains the familiar `<link>` and `<joint>` elements. This visually reinforces the scope difference.
3.  **Document Key Decisions**:
    *   `research.md` will be created to document the following:
        *   **XML Depth**: **Decision: Minimal XML syntax.** The chapter will show small, illustrative snippets but will avoid being a full XML schema tutorial. The focus is on what the tags *represent*, not memorizing the syntax.
        *   **Teaching Order**: **Decision: URDF-first.** Rationale: Learners are already familiar with URDF from Chapter 8. Introducing it first allows us to build on existing knowledge and then present SDF as an extension or "upgrade" for simulation.
        *   **Limitations vs. Workarounds**: **Decision: Focus on conceptual limitations.** For example, explain *that* URDF cannot express closed kinematic chains, but do not provide detailed workarounds, as this is an advanced topic. The goal is to help learners choose the right format, not to become expert URDF hackers.
        *   **Gazebo-Specifics**: **Decision: Only expose Gazebo behavior relevant to the format choice.** The key behavior to expose is the automatic conversion process and the use of `<gazebo>` tags within a URDF to provide SDF-specific information.
        *   **Examples**: **Decision: Primarily descriptive comparison with one hands-on example.** The chapter will use tables and diagrams for comparison. The single hands-on part will be to show the *same* URDF from a previous chapter being spawned in Gazebo, demonstrating the workflow.

**Output**: `specs/013-chapter12-robot-description-formats/research.md`

### Phase 1: Content Generation & Writing Plan

**Objective**: Draft the full chapter content, guiding the learner from URDF to SDF.

**Section-by-Section Writing Plan**:

1.  **Chapter Overview**: Frame the problem: "You know how to describe a robot with URDF. But to simulate it well, you need more. This chapter introduces SDF, the language of simulation."
2.  **What Is a Robot Description Format?**: A brief recap, reinforcing the idea of a formal, machine-readable representation of a robot's structure.
3.  **URDF**: Review the core components of URDF. Crucially, emphasize its strengths (kinematic representation, ROS ecosystem standard) and its **limitations** (no world description, no closed loops, limited sensor/physics modeling). This sets the stage for SDF.
4.  **SDF**: Introduce SDF as the solution to URDF's limitations. Use the comparison diagram to show its world-centric nature. Explain its key additional features: `<world>`, `<physics>`, `<sensor>`, `<plugin>`.
5.  **URDF vs SDF: Conceptual Comparison**: Present the detailed feature comparison table. This is the core of the chapter, directly comparing the two formats across key axes like expressiveness and simulation fidelity.
6.  **URDF and SDF in Gazebo**: This is the key practical section. Explain that Gazebo *natively* uses SDF. Walk through the **Robot Description Pipeline** diagram, explaining that when a user spawns a URDF, Gazebo performs a best-effort conversion in the background. Explain the role of the `<gazebo>` tag as a way to embed SDF-native information within a URDF file.
7.  **URDF and ROS 2 Integration**: Clarify that despite SDF's power, URDF remains the primary format for the core ROS 2 toolchain (`robot_state_publisher`, `tf2`). Emphasize that the two formats are complementary, not mutually exclusive. **Motto: "URDF for ROS, SDF for Gazebo."**
8.  **Modeling Humanoid Robots: Practical Guidelines**: Provide a simple decision framework: "Start with URDF. If you need to simulate advanced physics, multiple robots, or specific sensors, you will need to either add `<gazebo>` tags to your URDF or switch to a pure SDF file."

**Output**: Draft of the chapter's `.mdx` file.

### Phase 2: Quality Validation & Refinement

**Objective**: Ensure the comparison is accurate, the workflow is clear, and the concepts are not confusing.

**Quality Validation Checklist**:

- [ ] **Correctness**: Is the feature comparison between URDF and SDF accurate and up-to-date?
- [ ] **Interoperability**: Is the explanation of the URDF-to-SDF conversion process and the role of the `<gazebo>` tag correct?
- [ ] **Learner Clarity**: Does the chapter successfully create a clear mental model where URDF and SDF have distinct, complementary roles? Is the "Why do I need to know two formats?" question answered?
- [ ] **Alignment**: Does the chapter correctly reference the URDF from Chapter 8 and the Gazebo setup from Chapter 10?
- [ ] **Diagram Review**: Do the pipeline and comparison diagrams clearly and accurately convey the key concepts?

**Testing Strategy**:

1.  **Structural Validation**: A reviewer will take a standard humanoid URDF and manually create a corresponding SDF file, verifying that the link/joint hierarchy is consistent.
2.  **Conversion Path Validation**:
    *   Spawn a simple URDF (without `<gazebo>` tags) into Gazebo and note the default physics/sensor properties.
    *   Add a `<gazebo>` tag (e.g., to set a link's color or a sensor's noise). Re-spawn the robot and verify that the new property is correctly applied in the simulation. This validates the core workflow.
3.  **Semantic Consistency Check**: Ensure that concepts like "link," "joint," and "frame" are used consistently across this chapter and previous chapters (especially Chapter 8).

**Output**: Finalized, tested, and validated chapter content and diagrams.

## 4. Artifacts to be Generated

- `specs/013-chapter12-robot-description-formats/research.md`
- `specs/013-chapter12-robot-description-formats/plan.md` (this file)
- Draft content for the chapter's `.mdx` file.
- Descriptions for two diagrams:
  1.  Robot Description Pipeline (URDF → Gazebo → SDF)
  2.  Side-by-Side URDF vs. SDF Structure

## 5. Next Steps

- Proceed with creating the `research.md` file.
- Begin drafting the chapter content.
- Handoff to `/sp.tasks` to create specific writing, diagramming, and validation tasks.