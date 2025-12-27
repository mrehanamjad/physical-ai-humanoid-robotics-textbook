# Quickstart Guide: Setting up Docusaurus for Textbook Development

This guide provides instructions for setting up a local development environment to contribute to the Physical AI & Humanoid Robotics textbook using Docusaurus.

## Prerequisites

-   **Node.js**: Version 18.0 or higher.
-   **npm**: Node Package Manager (comes with Node.js).
-   **Git**: For version control.

## 1. Clone the Repository

First, clone the textbook repository to your local machine:

```bash
git clone [REPOSITORY_URL]
cd physical-ai-humanoid-robotics-textbook
```

Replace `[REPOSITORY_URL]` with the actual URL of the GitHub repository.

## 2. Install Dependencies

Navigate to the `textbook/` directory and install the Docusaurus project dependencies:

```bash
cd textbook/
npm install
```

## 3. Start Local Development Server

Once the dependencies are installed, you can start the Docusaurus local development server:

```bash
npm run start
```

This command will:
-   Build the Docusaurus site.
-   Start a local development server.
-   Open a new browser tab at `http://localhost:3000` (or another available port).

Any changes you make to the Markdown/MDX files will automatically hot-reload in your browser.

## 4. Explore the Content Structure

-   **Modules**: The main content modules are located under `textbook/docs/`. Each top-level directory within `docs/` represents a module.
-   **Chapters**: Each Markdown (`.md`) or MDX (`.mdx`) file within a module directory represents a chapter.
-   **Sidebar Navigation**: The sidebar structure is defined in `textbook/sidebars.ts`. Changes to this file will update the navigation.
-   **Docusaurus Configuration**: The main configuration for the site is in `textbook/docusaurus.config.ts`.

## 5. Adding New Content

To add a new chapter:
1.  Create a new Markdown (`.md`) or MDX (`.mdx`) file in the appropriate module directory (e.g., `textbook/docs/module1-ros2/`).
2.  Add Docusaurus front matter to the top of your file:

    ```markdown
    ---
    title: Your Chapter Title
    sidebar_position: [NUMBER] # Logical order within the module
    description: A brief description of the chapter content.
    ---
    ```
3.  Update `textbook/sidebars.ts` if the new chapter is not automatically picked up or if you need to adjust its position in the navigation.

## 6. Building for Production

To build the static HTML, CSS, and JavaScript files for production deployment:

```bash
npm run build
```

The static assets will be generated in the `textbook/build/` directory. These can then be deployed to any static site hosting service (e.g., GitHub Pages, Netlify, Vercel).
