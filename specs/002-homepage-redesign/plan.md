# Implementation Plan: Homepage Redesign

**Branch**: `002-homepage-redesign` | **Date**: 2025-12-18 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-homepage-redesign/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of homepage redesign to update branding, navigation, and accessibility. This includes updating the site title and tagline, removing blog functionality, redesigning the homepage with interactive module cards, improving text contrast in both light/dark modes, and updating navigation elements to better guide users to learning materials. The changes will focus on the Docusaurus-based educational website without modifying existing documentation files.

### Implementation Logic

The implementation will follow Docusaurus v3 conventions and use standard Docusaurus components and classes. For the "Start Learning" button styling, we'll use the built-in Docusaurus CSS classes 'button button--primary button--lg' which provide the solid blue robotic-themed appearance. The exact routing paths will ensure proper navigation to the documentation modules without "Page Not Found" errors.

### Final Polish Requirements

- Hero Section: In src/pages/index.tsx, the "Start Learning" button MUST use 'className="button button--primary button--lg"' to ensure it appears as a solid blue robotic-themed box.
- Module Cards: Link the 4 cards to their exact documentation paths:
    * Module 1 -> /docs/02-module-1-ros2/ros2-basics
    * Module 2 -> /docs/03-module-2-digital-twin/simulation
    * Module 3 -> /docs/04-module-3-nvidia-isaac/perception
    * Module 4 -> /docs/05-module-4-vla/vla-intelligence
- Footer & Navbar: In docusaurus.config.ts, fix the 'Tutorial' link and all 5 module links (Introduction + 4 Modules) to point to the correct .mdx files as seen in the docs/ folder structure.
- Exact Routing: Update all links in index.tsx, HomepageFeatures/index.tsx, and docusaurus.config.ts to use these exact relative paths:
    * Introduction: /docs/01-introduction/intro
    * Module 1: /docs/02-module-1-ros2/ros2-basics
    * Module 2: /docs/03-module-2-digital-twin/simulation
    * Module 3: /docs/04-module-3-nvidia-isaac/perception
    * Module 4: /docs/05-module-4-vla/vla-intelligence

## Technical Context

**Language/Version**: TypeScript/JavaScript (Docusaurus v3), Node.js 18+
**Primary Dependencies**: Docusaurus v3, React, Tailwind CSS or custom CSS, MDX
**Storage**: N/A (static site)
**Testing**: N/A (static site - manual verification)
**Target Platform**: Web (GitHub Pages deployment)
**Project Type**: Web application (static site)
**Performance Goals**: <3 second homepage load time, WCAG AA contrast compliance
**Constraints**: Must maintain compatibility with existing documentation structure, no changes to /docs MDX files
**Scale/Scope**: Single educational website with multiple learning modules

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Technical Precision**: Implementation will ensure all UI elements function correctly and maintain accessibility standards
2. **Spec-Driven Consistency**: All changes will strictly follow the requirements in the feature specification
3. **AI-Native Education**: Homepage will maintain clear navigation to educational content
4. **Source of Truth**: Docusaurus configuration will be updated while preserving MDX content as source of truth
5. **RAG Contextual Grounding**: No changes to RAG functionality, only presentation layer
6. **Separation of Concerns**: Changes will be limited to frontend presentation (Docusaurus config, homepage, CSS) without touching backend logic
7. **Documentation Standards**: Will maintain MDX format for content and Mermaid.js for diagrams (no changes needed)
8. **Code Quality Requirements**: CSS will follow best practices for accessibility and theming
9. **Technology Stack Requirements**: Implementation will use Docusaurus v3 features as specified

## Project Structure

### Documentation (this feature)

```text
specs/002-homepage-redesign/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure
src/
├── pages/
│   └── index.tsx        # Homepage redesign
├── css/
│   └── custom.css       # Updated styling with high contrast themes
└── components/
    └── ModuleCard.tsx   # Interactive module cards component

# Docusaurus configuration
docusaurus.config.ts      # Updated site title, tagline, and navigation

# Documentation (unchanged)
docs/
├── 01-introduction/
├── 02-module-1-ros2/
├── 03-module-2-digital-twin/
├── 04-module-3-nvidia-isaac/
└── 05-module-4-vla/

# Blog (to be deleted)
blog/                     # Will be removed as part of cleanup
```

**Structure Decision**: Selected web application structure with focus on Docusaurus-based static site. The implementation will modify the homepage, CSS, and Docusaurus configuration while preserving the existing documentation structure in the /docs directory.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |
