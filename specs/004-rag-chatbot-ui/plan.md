# Implementation Plan: RAG Chatbot UI

**Branch**: `004-rag-chatbot-ui` | **Date**: 2025-12-21 | **Spec**: [specs/004-rag-chatbot-ui/spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-rag-chatbot-ui/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a React-based chatbot UI for the Docusaurus documentation site that connects to an existing FastAPI/Qdrant RAG backend. The UI includes a floating action button (FAB) for access, a scrollable chat window with Blue/Slate AI/Robotics theme, and integration with react-markdown for proper rendering of technical content including code blocks. The UI will connect to backend endpoints at /chat/start and /chat/{token}/message, display source references as clickable badges, and be globally integrated via src/theme/Root.js.

## Technical Context

**Language/Version**: TypeScript/JavaScript for React components, Docusaurus v3
**Primary Dependencies**: React, react-markdown, Docusaurus v3, FastAPI (backend)
**Storage**: N/A (UI only, backend handles storage)
**Testing**: Jest for unit testing, React Testing Library for component testing
**Target Platform**: Web browser, responsive design for all screen sizes
**Project Type**: Web application (frontend component for Docusaurus site)
**Performance Goals**: <1 second UI load time, <5 seconds for response display, smooth scrolling for chat history
**Constraints**: Must integrate with existing Docusaurus site structure, follow Blue/Slate AI/Robotics design theme, maintain compatibility with ROS2/Gazebo/Isaac Sim documentation
**Scale/Scope**: Site-wide availability across all documentation pages, support for concurrent users, up to 50 messages in chat history

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Technical Precision**: The UI implementation must maintain technical accuracy in displaying robotics content, ensuring code blocks and technical formatting are preserved through react-markdown integration (PASSED - requirement specified).

2. **Spec-Driven Consistency**: This implementation plan strictly follows the feature specification with specific requirements for component structure, API integration, and design theme (PASSED - all spec requirements addressed).

3. **AI-Native Education**: The chatbot UI demonstrates AI concepts through practical implementation with RAG integration, providing hands-on learning experience (PASSED - core functionality implemented).

4. **Source of Truth**: The UI will properly display content from the Docusaurus Markdown files that serve as the primary source of truth for both website and RAG vector database (PASSED - markdown rendering requirement included).

5. **RAG Contextual Grounding**: The UI connects to backend that prioritizes contextual grounding, displaying responses based only on textbook content (PASSED - backend handles this per constitution).

6. **Separation of Concerns**: The UI components will be properly separated in /src (RAG logic) vs /docs (book content), maintaining clean organization (PASSED - component location specified as src/components/ChatBot/).

7. **Documentation Standards**: The UI will support Docusaurus v3 features and properly render MDX content with technical formatting (PASSED - react-markdown integration specified).

8. **Code Quality Requirements**: The React components will follow appropriate standards for the JavaScript/TypeScript ecosystem (PASSED - testing strategy defined).

9. **Technology Stack Requirements**: Implementation aligns with specified stack: Docusaurus v3 frontend with connection to existing FastAPI backend (PASSED - architecture matches).

10. **Security and Deployment**: UI implementation will not include API keys or sensitive data, maintaining security standards (PASSED - UI only, no sensitive data storage).

## Project Structure

### Documentation (this feature)

```text
specs/004-rag-chatbot-ui/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   └── ChatBot/              # Chatbot UI components
│       ├── ChatWindow.tsx    # Main chat window component
│       ├── MessageBubble.tsx # Individual message display
│       ├── FloatingActionButton.tsx # FAB for chat access
│       ├── ChatInput.tsx     # Input field for user messages
│       ├── SourceBadge.tsx   # Component for source references
│       ├── LoadingSkeleton.tsx # Skeleton UI for loading states
│       └── styles.module.css # Blue/Slate theme styles
├── services/
│   └── chatbot-api.ts        # API service for backend communication
└── theme/
    └── Root.js               # Global wrapper for site-wide availability
```

**Structure Decision**: Web application frontend components integrated into existing Docusaurus structure. All chatbot UI components placed in src/components/ChatBot/ as required by specification, with API service in src/services/ and global integration via src/theme/Root.js.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
