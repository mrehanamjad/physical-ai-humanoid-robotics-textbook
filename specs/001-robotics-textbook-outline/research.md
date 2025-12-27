# Research Findings: Docusaurus and Context7 MCP Usage

## Decision: Docusaurus as the documentation framework

## Rationale:

The choice of Docusaurus as the documentation framework for the "Physical AI & Humanoid Robotics" textbook aligns strongly with the project's constitution and specific requirements, particularly due to its robust features for technical content, versioning, extensibility, and open-source nature.

1.  **Versioning**: Docusaurus excels in managing different versions of documentation, which is crucial for a technical textbook that will likely have multiple editions and updates.
2.  **Extensibility & Customization**: Built on React, Docusaurus allows for deep customization and the creation of interactive elements, which is invaluable for embedding live code examples or simulations.
3.  **"Docs-as-Code" Workflow**: It integrates seamlessly with Git and GitHub, treating documentation like code. This workflow is ideal for technical authors and contributors.
4.  **Features**: Out-of-the-box support for search, internationalization, and a robust plugin ecosystem further contribute to a high-quality learning experience.
5.  **Open Source & Free**: As a free, open-source static site generator, Docusaurus fully adheres to the "Open Knowledge" principle.

## Alternatives Considered:

-   **MkDocs**: While simple and easy to use, it may require more manual configuration for advanced features that Docusaurus offers natively.
-   **GitBook**: Primarily a cloud-based service, which may conflict with the "Open Knowledge" principle due to its proprietary nature and potential costs.

## Decision: Context7 MCP for Documentation Verification

## Rationale:

The Context7 MCP server will be used as an optional tool to ensure the correctness and consistency of the Docusaurus implementation. It acts as a verification layer, providing a way to query and validate against the official Docusaurus documentation. This approach supports the "Technical Correctness" and "Reproducibility & Traceability" principles of the project's constitution.

### Specific Scenarios and Queries:

The Context7 MCP server will be invoked when there is uncertainty about Docusaurus features, APIs, or best practices. Example queries include:

1.  **Validating Front Matter Fields**:
    -   "What are the valid fields for Docusaurus front matter in version X.X?"
    -   "Show me an example of how to use the `sidebar_position` front matter field."
    -   "Does Docusaurus support custom front matter fields? If so, how are they configured?"

2.  **Confirming MDX Syntax and Component Usage**:
    -   "Provide an example of how to import and use a React component in an MDX file."
    -   "What is the correct syntax for creating an admonition in Docusaurus?"
    -   "How can I create tabs in a Docusaurus document using MDX?"

3.  **Verifying Docusaurus Conventions**:
    -   "What is the recommended way to structure the sidebar in `sidebars.ts` for a multi-level navigation?"
    -   "How does Docusaurus handle asset paths for images and other static files?"
    -   "What are the best practices for cross-referencing other documents within a Docusaurus site?"

By using the Context7 MCP server for these types of queries, we can reduce the time spent on manual documentation lookups and ensure that the textbook adheres to the latest Docusaurus standards.
