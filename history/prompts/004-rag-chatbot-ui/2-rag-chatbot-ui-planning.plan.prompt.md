---
id: 2
title: rag-chatbot-ui-planning
stage: plan
date: '2025-12-21'
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot-ui
branch: 004-rag-chatbot-ui
user: Claude Code
command: /sp.plan
labels:
  - planning
  - chatbot
  - ui
  - docusaurus
  - react
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-chatbot-ui/plan.md
  - specs/004-rag-chatbot-ui/research.md
  - specs/004-rag-chatbot-ui/data-model.md
  - specs/004-rag-chatbot-ui/quickstart.md
  - specs/004-rag-chatbot-ui/contracts/chat-api-contract.md
tests: []
---

# RAG Chatbot UI Planning

## Prompt

```
"Phase 004: RAG Chatbot UI Implementation.

Setup ChatBot component folder in 'src/components/ChatBot/'.

Create the ChatWindow and MessageBubble sub-components with the Blue/Slate Robotics theme.

Implement the API service logic to handle /chat/start (on mount) and /chat/{token}/message (on send).

Integrate 'react-markdown' to ensure the robotics book's code blocks and formatting are preserved.

Map 'sources' from the API response to clickable UI badges under each message.

Update 'src/theme/Root.js' to wrap the entire Docusaurus site, making the chatbot available on every page (ROS2, Gazebo, Isaac Sim, etc.).

Verify the UI is responsive and includes loading skeletons for the 'Thinking' state."
```

## Response

Created comprehensive implementation plan for RAG Chatbot UI with the following artifacts:

1. **Implementation Plan** (plan.md): Complete technical context, constitution check, and project structure
2. **Research** (research.md): Decisions on React components, API integration, markdown rendering, and UI patterns
3. **Data Model** (data-model.md): Entity definitions for ChatSession, ChatMessage, SourceReference, ChatUIState, and APIConfig
4. **API Contracts** (contracts/chat-api-contract.md): Contract for /chat/start and /chat/{token}/message endpoints
5. **Quickstart Guide** (quickstart.md): Setup instructions and development workflow

The plan includes React functional components with TypeScript, react-markdown for technical content rendering, CSS Modules for styling with Blue/Slate theme, and proper API integration with the existing FastAPI backend. The UI will be globally integrated via Docusaurus Root.js component.