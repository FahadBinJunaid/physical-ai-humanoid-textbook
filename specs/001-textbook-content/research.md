# Research: Physical AI & Humanoid Robotics Textbook Content

## Decision: Docusaurus v3 with TypeScript Template
**Rationale**: The specification requires Docusaurus v3 with TypeScript compatibility. This is the current stable version that supports MDX files and provides the best ecosystem for documentation sites with component integration capabilities.

**Alternatives considered**:
- Docusaurus v2: Would be outdated and miss newer features
- VuePress: Different ecosystem with less robotics community adoption
- GitBook: More limited customization options

## Decision: MDX File Format for All Content
**Rationale**: MDX allows for both Markdown content and React component integration, which is essential for the future ChatBot component integration and interactive elements. TypeScript compatibility ensures proper typing for any components used.

**Alternatives considered**:
- Pure Markdown: Would not support component integration
- JSX files: Would lose Markdown simplicity for content authors

## Decision: Python for ROS2 Code Examples
**Rationale**: ROS2 has strong Python support and Python is more beginner-friendly for students learning robotics concepts. The specification mentions Python/ROS2 code blocks specifically.

**Alternatives considered**:
- C++ for ROS2: More performant but steeper learning curve for students
- Both Python and C++: Would add complexity without clear benefit for learning

## Decision: File Structure with Numbered Directories (01-, 02-, etc.)
**Rationale**: The specification explicitly requires the numbered directory structure to ensure correct sidebar ordering. This provides clear progression through the content.

**Alternatives considered**:
- Alphabetical naming: Would not guarantee proper ordering
- Nested structure: Would complicate navigation

## Decision: Docusaurus Front-matter Requirements
**Rationale**: The specification requires id, title, and sidebar_position for each MDX file to properly integrate with Docusaurus navigation system.

**Implementation**: Each file will include:
```md
---
id: module-name
title: Module Title
sidebar_position: N
---
```

## Decision: ChatBot Component Placeholder
**Rationale**: The specification requires `// TODO: Import <ChatBot /> component here` at the bottom of each module for future RAG integration.

**Implementation**: Simple comment placeholder that can be replaced with actual component later.

## Decision: Technical Content Standards
**Rationale**: Content must follow 2024-2025 industry standards as per the project constitution (Technical Precision principle). This ensures educational material remains current and practically applicable.

**Implementation**: Focus on ROS 2 Jazzy/Humble, Gazebo Harmonic, and NVIDIA Isaac Sim/ROS as specified in the constitution.