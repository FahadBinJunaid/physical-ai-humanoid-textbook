# Quickstart: RAG Chatbot UI Development

**Feature**: 004-rag-chatbot-ui
**Date**: 2025-12-21
**Status**: Complete

## Prerequisites

- Node.js 18+ installed
- Docusaurus v3 project already set up
- Backend service running at http://127.0.0.1:8000
- React and TypeScript development environment

## Setup Steps

### 1. Install Dependencies
```bash
npm install react-markdown @mdx-js/react rehype-highlight
```

### 2. Create Component Directory Structure
```bash
mkdir -p src/components/ChatBot
```

### 3. Create UI Components
Create the following files in src/components/ChatBot/:
- ChatWindow.tsx
- MessageBubble.tsx
- FloatingActionButton.tsx
- ChatInput.tsx
- SourceBadge.tsx
- LoadingSkeleton.tsx
- styles.module.css

### 4. Create API Service
Create file at src/services/chatbot-api.ts with functions for:
- startNewSession(): Promise<ChatSession>
- sendMessage(token: string, message: string): Promise<ChatResponse>

### 5. Global Integration
Update src/theme/Root.js to include the chatbot component wrapper

### 6. Run Development Server
```bash
npm run start
```

## Development Workflow

1. Start backend service: `cd chatbot-backend && uvicorn main:app --reload`
2. Start Docusaurus: `npm run start`
3. Access chatbot via the floating action button on any page

## Testing

### Unit Tests
- Component rendering tests
- API service integration tests
- State management tests

### Integration Tests
- End-to-end chat flow
- API communication
- Source reference display

## Deployment

1. Ensure backend API is accessible in production environment
2. Build Docusaurus site: `npm run build`
3. Deploy to GitHub Pages or preferred hosting
4. Verify chatbot functionality across all site pages