# Data Model: Book Content Outline

## Entities

### Module

-   **Description**: Represents a major section of the textbook, containing a collection of chapters focused on a specific theme. Modules are the highest level of organization for the textbook content.
-   **Attributes**:
    -   `title` (string): The title of the module (e.g., "The Robotic Nervous System (ROS 2)").
    -   `short_name` (string): A short, slug-like name for the module, used for file paths and internal referencing (e.g., "ros2-nervous-system").
    -   `description` (string, optional): A brief overview of the module's content and learning objectives.
    -   `chapters` (list of Chapter): A ordered list of chapters belonging to this module.

### Chapter

-   **Description**: Represents a single content unit within a module, with a title that describes its specific learning objectives. Chapters are the primary building blocks of the textbook content.
-   **Attributes**:
    -   `title` (string): The title of the chapter (e.g., "Physical AI Foundations and Embodied Intelligence").
    -   `file_name` (string): The filename for the Markdown/MDX content of the chapter (e.g., "01-physical-ai-foundations.md").
    -   `description` (string, optional): A brief overview of the chapter's content and learning outcomes.
    -   `sidebar_position` (integer): An integer representing the chapter's position within its module's sidebar navigation.
    -   `module` (Module): A reference to the parent module this chapter belongs to.

## Relationships

-   **Module has many Chapters**: A one-to-many relationship where each Module can contain multiple Chapters. Each Chapter belongs to exactly one Module.
