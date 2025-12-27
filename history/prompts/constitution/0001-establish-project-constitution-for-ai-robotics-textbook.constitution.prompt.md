---
id: 0001
title: Establish Project Constitution for AI Robotics Textbook
stage: constitution
date: 2025-12-19
surface: agent
model: gemini
feature: none
branch: 001-book-layout
user: Your Name
command: /sp.constitution
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - none
---

## Prompt

/sp.constitution /sp.constitution

Project: 
AI-Native Textbook for Teaching Physical AI & Humanoid Robotics with an Integrated RAG Chatbot

Purpose:
Create a publicly accessible, continuously improvable textbook that teaches Physical AI and Humanoid Robotics using AI-native, specification-driven workflows, and augments learning with an embedded Retrieval-Augmented Generation (RAG) chatbot.

Core principles:
- Specification-Driven Development (all content, features, and workflows must be derived from explicit specs)
- Technical accuracy grounded in robotics, AI, and control systems fundamentals
- Pedagogical clarity for students with CS / engineering backgrounds
- AI-native authoring (human-in-the-loop, agent-assisted writing and review)
- Open knowledge (free, public, and version-controlled)
- Reproducibility and traceability of concepts, code, and references

Target audience:
- Undergraduate and graduate students in Computer Science, AI, Robotics, and Mechatronics
- Educators teaching Physical AI or Humanoid Robotics
- Developers transitioning from software AI to embodied intelligence

Key standards:
- All technical explanations must be conceptually correct and internally consistent
- Robotics and AI concepts must be explained from first principles before advanced usage
- Mathematical formulations must be accompanied by intuitive explanations
- Code snippets must be runnable, minimal, and well-commented
- Diagrams and illustrations should clarify embodiment, sensing, control, and learning loops
- RAG chatbot responses must be grounded strictly in the book’s indexed content
- Selected-text Q&A must only reference the user-selected passage

Technology standards:
- Documentation framework: Docusaurus
- Version control & hosting: GitHub + GitHub Pages
- Spec-driven authoring: Spec-Kit Plus
- AI coding & writing assistant: Claude Code
- Backend API: FastAPI
- LLM integration: OpenAI Agents / ChatKit SDKs
- Vector database: Qdrant Cloud (Free Tier)
- Relational database: Neon Serverless Postgres
- Deployment must be reproducible via documented setup steps

Content scope:
- Foundations of Physical AI
- Humanoid robot morphology and kinematics
- Sensors, perception, and multimodal input
- Control systems and motion planning
- Learning in embodied agents (RL, IL, hybrid approaches)
- Simulation vs real-world transfer (sim2real)
- Ethics, safety, and responsible humanoid AI
- Case studies and practical examples

Constraints:
- Writing style: clear, structured, and instructional
- Tone: academic-friendly but accessible
- All chapters must follow a consistent template
- Avoid unsupported claims or speculative hype
- No proprietary or paywalled content excerpts

Quality & validation rules:
- Every chapter must pass internal consistency checks
- Terminology must be defined before use
- Cross-references between chapters must be accurate
- RAG chatbot answers must cite chapter/section sources
- No hallucinated references or undocumented assumptions

Success criteria:
- Book is fully deployed and publicly accessible
- Readers can learn Physical AI concepts end-to-end using the book alone
- Embedded chatbot accurately answers questions about the content
- Selected-text Q&A works as intended with strict grounding
- Project demonstrates clear application of Specification-Driven Development
- Repository is understandable, reproducible, and hackathon-ready

## Response snapshot

I have updated the project constitution and ensured its principles are reflected in the project's templates.

## Outcome

- ✅ Impact: Project constitution was established and dependent templates were updated for consistency.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md, .specify/templates/plan-template.md
- 🔁 Next prompts: Implement the feature specification based on the updated constitution. I want to build...
- 🧠 Reflection: The constitution provides a strong foundation for the project. The principle-driven approach will guide development.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none