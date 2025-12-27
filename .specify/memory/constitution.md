<!--
Sync Impact Report:
- Version change: 0.1.0 → 1.0.1
- Improvements: Language precision, enforceability, and governance clarity
- No scope expansion; intent preserved
-->

# Constitution  
## AI-Native Textbook for Teaching Physical AI & Humanoid Robotics  
### with an Integrated Retrieval-Augmented Generation (RAG) Chatbot

---

## 1. Purpose

This project exists to create a **public, AI-native, specification-driven textbook** for teaching **Physical AI and Humanoid Robotics**, augmented with an embedded **Retrieval-Augmented Generation (RAG) chatbot** that enables grounded, interactive learning directly from the book’s content.

The textbook shall serve as both:
- A **complete learning resource** for embodied intelligence, and
- A **reference implementation** of Specification-Driven Development applied to AI-native educational systems.

---

## 2. Core Principles

### I. Specification-Driven Development (SDD)
All content, system behavior, infrastructure, and workflows **must originate from explicit, written specifications**.  
No feature, chapter, or behavior may exist without a corresponding spec.

### II. Technical Correctness
All material must be grounded in established principles of:
- Robotics
- Artificial Intelligence
- Control systems
- Machine learning for embodied agents  

Incorrect, misleading, or oversimplified explanations are unacceptable.

### III. Pedagogical Clarity
Content must be written for readers with **computer science or engineering backgrounds**, prioritizing:
- Conceptual understanding
- Progressive difficulty
- Clear learning outcomes

### IV. AI-Native Authoring
AI tools are first-class collaborators.  
All writing, coding, and review processes are **human-in-the-loop**, with AI agents used for drafting, validation, refactoring, and consistency checks.

### V. Open Knowledge
The textbook and all supporting infrastructure must remain:
- Free
- Publicly accessible
- Version-controlled  

Closed, proprietary, or paywalled dependencies are prohibited.

### VI. Reproducibility & Traceability
Every concept, diagram, code snippet, and chatbot response must be:
- Traceable to source content
- Reproducible using documented steps
- Verifiable by readers and contributors

---

## 3. Target Audience

- Undergraduate and graduate students in:
  - Computer Science
  - Artificial Intelligence
  - Robotics
  - Mechatronics
- Educators teaching Physical AI or Humanoid Robotics
- Software engineers transitioning to embodied and physical intelligence systems

---

## 4. Key Standards

- All explanations must be internally consistent and technically sound.
- Concepts must be introduced from first principles before advanced application.
- Mathematical formulations must include intuitive, plain-language explanations.
- Code examples must be:
  - Runnable
  - Minimal
  - Well-commented
- Visuals must clarify:
  - Embodiment
  - Sensing
  - Control loops
  - Learning pipelines
- The RAG chatbot must:
  - Answer only using indexed book content
  - Cite exact chapters or sections
  - Respect selected-text-only constraints when applicable

---

## 5. Technology Standards

The project must use the following stack unless formally amended:

- Documentation: **Docusaurus**
- Version control & hosting: **GitHub + GitHub Pages**
- Specification framework: **Spec-Kit Plus**
- AI coding & authoring: **Claude Code**
- Backend API: **FastAPI**
- LLM orchestration: **OpenAI Agents / ChatKit SDKs**
- Vector database: **Qdrant Cloud (Free Tier)**
- Relational database: **Neon Serverless Postgres**

All deployments must be reproducible from documented setup instructions.

---

## 6. Content Scope

The textbook must cover, at minimum:

- Foundations of Physical AI
- Humanoid robot morphology and kinematics
- Sensors, perception, and multimodal input
- Control systems and motion planning
- Learning in embodied agents:
  - Reinforcement Learning
  - Imitation Learning
  - Hybrid approaches
- Simulation-to-real-world transfer (Sim2Real)
- Ethics, safety, and responsible humanoid AI
- Practical case studies and applied examples

---

## 7. Constraints

- Writing must be structured, instructional, and precise.
- Tone must remain academic yet approachable.
- Every chapter must follow a standardized template.
- Unsupported claims, speculation, and hype are forbidden.
- Proprietary or paywalled content may not be included or quoted.

---

## 8. Quality & Validation Rules

- Every chapter must pass:
  - Internal consistency checks
  - Terminology validation
- All terms must be defined before first use.
- Cross-references must be accurate and maintained.
- Chatbot responses must always cite source sections.
- Hallucinated references, assumptions, or outputs are strictly prohibited.

---

## 9. Success Criteria

This project is considered successful only if:

- The textbook is fully deployed and publicly accessible.
- A reader can learn Physical AI end-to-end using the book alone.
- The embedded chatbot answers questions accurately and with strict grounding.
- Selected-text Q&A behaves correctly and defensively.
- The project demonstrates clear, disciplined use of Specification-Driven Development.
- The repository is understandable, reproducible, and hackathon-ready.

---

## 10. Governance

This Constitution is the **single source of truth** for the project.

- All contributions must comply with it.
- All pull requests must explicitly confirm compliance.
- Amendments require:
  - A written proposal
  - Review by core contributors
  - Majority approval

No contribution supersedes this Constitution.

---

**Version**: 1.0.1  
**Ratified**: 2025-12-19  
**Last Amended**: 2025-12-19
