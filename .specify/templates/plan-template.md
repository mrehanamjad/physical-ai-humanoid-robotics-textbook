# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION]  
**Primary Dependencies**: [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION]  
**Storage**: [if applicable, e.g., PostgreSQL, CoreData, files or N/A]  
**Testing**: [e.g., pytest, XCTest, cargo test or NEEDS CLARIFICATION]  
**Target Platform**: [e.g., Linux server, iOS 15+, WASM or NEEDS CLARIFICATION]
**Project Type**: [single/web/mobile - determines source structure]  
**Performance Goals**: [domain-specific, e.g., 1000 req/s, 10k lines/sec, 60 fps or NEEDS CLARIFICATION]  
**Constraints**: [domain-specific, e.g., <200ms p95, <100MB memory, offline-capable or NEEDS CLARIFICATION]  
**Scale/Scope**: [domain-specific, e.g., 10k users, 1M LOC, 50 screens or NEEDS CLARIFICATION]

## Constitution Check

*GATE: This plan MUST be fully compliant with the project constitution. All checks must pass.*

### I: Core Principles
- **[ ] 1. Clear, Progressive Learning:** Does the plan outline a logical learning path? (See Const. I.1)
- **[ ] 2. Hands-On, Practical Approach:** Does the plan include concrete examples and exercises? (See Const. I.2)
- **[ ] 3. Technical Accuracy:** Is there a process for verifying technical accuracy and citing sources? (See Const. I.3)
- **[ ] 4. Visual Learning:** Does the plan incorporate diagrams and visual aids? (See Const. I.4)
- **[ ] 5. Educational Clarity:** Is the content plan focused on accessibility and clarity for the target audience? (See Const. I.5)
- **[ ] 6. Ethical Responsibility:** Are safety and ethical considerations addressed in the plan? (See Const. I.6)

### II: Key Standards
- **[ ] 1. Content Format:** Does the plan specify Docusaurus-compatible Markdown/MDX? (See Const. II.1)
- **[ ] 2. Code Examples:** Are code quality standards (PEP 8, docstrings, testing) included in the plan? (See Const. II.2)
- **[ ] 3. Content Structure:** Does the feature plan adhere to the required chapter structure? (See Const. II.3)
- **[ ] 4. Accessibility and Responsiveness:** Is there a plan to meet WCAG 2.1 AA and responsive design standards? (See Const. II.4)
- **[ ] 5. Development Workflow:** Does the plan align with the Git workflow, PR process, and CI/CD strategy? (See Const. II.5)
- **[ ] 6. Visual Design:** Are visual design standards for diagrams and typography considered? (See Const. II.6)

### III: Constraints & IV: Success Criteria
- **[ ] Scope:** Does the plan respect the defined content scope? (See Const. III.1)
- **[ ] Prerequisites:** Are the target audience's prerequisites properly considered? (See Const. III.3)
- **[ ] Learning Outcomes:** Does the plan contribute to the defined learning outcomes? (See Const. IV.1)
- **[ ] Technical Deliverables:** Does the plan align with the required technical deliverables? (See Const. IV.2)

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
