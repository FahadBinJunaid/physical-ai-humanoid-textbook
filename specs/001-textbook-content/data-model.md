# Data Model: Physical AI & Humanoid Robotics Textbook Content

## Module Content Entity

**Description**: Represents a single educational module in the robotics textbook

**Attributes**:
- `id`: String - Unique identifier for the module (e.g., "intro", "ros2-basics", "simulation")
- `title`: String - Display title for the module
- `sidebar_position`: Number - Position in the navigation sidebar (1, 2, 3, etc.)
- `content`: String - The MDX content of the module
- `file_path`: String - Path to the MDX file (e.g., "/docs/01-introduction/intro.mdx")
- `prerequisites`: Array<String> - List of prerequisite modules
- `learning_objectives`: Array<String> - Learning objectives for the module
- `code_examples`: Array<Object> - Code examples included in the module
  - `language`: String - Programming language (python, xml for URDF, etc.)
  - `code`: String - The actual code snippet
  - `description`: String - Explanation of the code example

## Documentation File Entity

**Description**: Represents an MDX file containing educational content

**Attributes**:
- `file_name`: String - Name of the file (e.g., "intro.mdx")
- `file_path`: String - Full path from repository root
- `front_matter`: Object - Docusaurus front-matter containing id, title, sidebar_position
- `content_type`: String - Type of content (introduction, tutorial, reference, etc.)
- `module_number`: Number - Sequential number for ordering (01, 02, 03, etc.)
- `module_category`: String - Category (introduction, ros2, simulation, perception, vla)

## Navigation Structure Entity

**Description**: Represents the hierarchical organization of content for sidebar navigation

**Attributes**:
- `sidebar_id`: String - Unique identifier for the sidebar
- `items`: Array<Object> - Navigation items in the sidebar
  - `type`: String - Type of item (doc, link, category)
  - `id`: String - Reference to the document id
  - `label`: String - Display label for the navigation item
  - `position`: Number - Order position in the sidebar

## Validation Rules

1. **File Path Validation**: Each module must have a unique file_path that follows the pattern `/docs/{numbered_prefix}-{category}/{filename}.mdx`
2. **Front Matter Validation**: Every MDX file must contain valid Docusaurus front-matter with id, title, and sidebar_position
3. **Sequential Ordering**: Module numbers must be sequential (01, 02, 03, 04, 05)
4. **Content Completeness**: Each module must contain educational content appropriate for its category
5. **Prerequisite Validation**: Modules should not have prerequisites that come after them in sequence

## State Transitions

**Content Creation Workflow**:
- `planned` → `draft` → `review` → `published`
- Each state represents the maturity of the educational content
- Only published content is included in the final textbook

**Relationships**:
- Module Content "belongs to" Documentation File (one-to-one)
- Documentation Files "are organized by" Navigation Structure (many-to-one)
- Module Content "has prerequisites" of other Module Content (many-to-many)