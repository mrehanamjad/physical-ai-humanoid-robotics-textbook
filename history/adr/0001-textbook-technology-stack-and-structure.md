# ADR-0001: Textbook Technology Stack and Structure

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-book-layout
- **Context:** The project requires a robust, maintainable, and scalable platform for creating and hosting the "Physical AI & Humanoid Robotics Course Textbook." The chosen solution must provide an excellent authoring experience for technical content, support visual aids and code examples, and result in a fast, accessible, and modern website for learners.

## Decision

We will use **Docusaurus v3.x** as the static site generator for the textbook. The content will be written in **Markdown/MDX**. The project will follow a standard Docusaurus project structure, with the textbook content located in the `/docs` directory, organized into modules and chapters.

The core technology stack is:
-   **Framework**: Docusaurus v3.x with React v18.x
-   **Language**: Typescript, Python 3.10+ for code examples
-   **Styling**: Infima (Docusaurus's default styling), with custom CSS in `/src/css`.
-   **Diagrams**: Mermaid.js for all diagrams to ensure consistency.
-   **Deployment**: GitHub Pages, managed via GitHub Actions for CI/CD.
-   **Primary Dependencies**: ROS 2 (Humble/Iron), Gazebo (Fortress/Ignition), NVIDIA Isaac Sim.

## Consequences

### Positive

-   **Excellent Authoring Experience**: Docusaurus is optimized for documentation, providing features like MDX, versioning, and search out of the box.
-   **Fast & Modern Website**: The resulting site is a fast, single-page React application, which is good for user experience.
-   **Markdown-centric**: Allows authors to focus on content. Code examples and diagrams can be embedded directly.
-   **Extensible**: Docusaurus can be extended with custom React components, allowing for interactive examples and visualizations.
-   **Strong Community & Ecosystem**: Docusaurus is widely used and has a large community, making it easy to find support and plugins.

### Negative

-   **Node.js Dependency**: The project is dependent on the Node.js and React ecosystem, requiring familiarity with these tools for advanced customization.
-   **Build Step Required**: Content changes require a build step to be visible in production, though a dev server provides hot-reloading.
-   **Less Flexible than a Custom App**: While extensible, Docusaurus imposes a certain structure and set of conventions.

## Alternatives Considered

-   **Custom React/Next.js Application**:
    -   **Why Rejected**: This would provide maximum flexibility but would require significant effort to build features that Docusaurus provides for free (e.g., routing, sidebars, search, versioning). The primary focus should be on content creation, not web development.
-   **Other Static Site Generators (e.g., Hugo, Jekyll)**:
    -   **Why Rejected**: While fast, Hugo and Jekyll have less seamless integration with React for creating interactive components. Docusaurus's tight integration with React is a key advantage for a technical textbook that will benefit from interactive examples.
-   **GitBook**:
    -   **Why Rejected**: GitBook is a hosted platform that offers less control over the final output and styling. A self-hosted Docusaurus site provides more ownership and customization options. It also avoids vendor lock-in.

## References

-   **Feature Spec**: [/specs/001-book-layout/spec.md](/specs/001-book-layout/spec.md)
-   **Implementation Plan**: [/specs/001-book-layout/plan.md](/specs/001-book-layout/plan.md)
-   **Related ADRs**: None
-   **Evaluator Evidence**: [/history/prompts/001-book-layout/0002-create-technical-plan-for-book-layout.plan.prompt.md](/history/prompts/001-book-layout/0002-create-technical-plan-for-book-layout.plan.prompt.md)
