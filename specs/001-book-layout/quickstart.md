# Quickstart Guide

**Date**: 2025-12-07
**Status**: Completed

This guide provides the basic steps to set up the development environment and start contributing to the textbook.

## Prerequisites

-   Node.js v20.x LTS
-   npm or yarn
-   Git

## Setup and Installation

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd physical-ai-humanoid-robotics-textbook
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    # or
    yarn install
    ```

## Running the Development Server

To start the Docusaurus development server and view the textbook locally:

```bash
npm run start
# or
yarn start
```

This will open a browser window at `http://localhost:3000`. The site will automatically reload as you make changes to the content in the `/docs` directory.

## Building the Project

To create a static build of the website for deployment:

```bash
npm run build
# or
yarn build
```

The output will be in the `build` directory.
