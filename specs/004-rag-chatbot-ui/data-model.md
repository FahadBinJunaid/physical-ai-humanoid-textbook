# Data Model: RAG Chatbot UI

**Feature**: 004-rag-chatbot-ui
**Date**: 2025-12-21
**Status**: Complete

## Entities

### ChatSession
**Description**: Represents a conversation between user and bot
**Fields**:
- id: string (unique session identifier from backend)
- token: string (session token for API communication)
- createdAt: Date (when session was initiated)
- isActive: boolean (whether session is currently active)

### ChatMessage
**Description**: Represents a single exchange in the conversation
**Fields**:
- id: string (unique message identifier)
- sessionId: string (reference to parent session)
- sender: 'user' | 'bot' (who sent the message)
- content: string (the message content)
- timestamp: Date (when message was created)
- sources: SourceReference[] (optional source references for bot responses)
- status: 'pending' | 'sent' | 'delivered' | 'error' (message status for UI)

### SourceReference
**Description**: Represents a link to original documentation that supports the bot's response
**Fields**:
- id: string (unique identifier for the source)
- title: string (display title for the source)
- url: string (URL to the original documentation)
- page: string (page or section name)
- relevance: number (optional relevance score from 0-1)

### ChatUIState
**Description**: Represents the visibility and position state of the chat window
**Fields**:
- isOpen: boolean (whether chat window is visible)
- isMinimized: boolean (whether chat window is minimized)
- position: { x: number, y: number } (coordinates for window positioning)
- unreadCount: number (number of unread messages)
- lastActive: Date (when chat was last interacted with)

### APIConfig
**Description**: Configuration for backend API communication
**Fields**:
- baseUrl: string (base URL for API endpoints)
- startEndpoint: string (endpoint for starting new sessions)
- messageEndpoint: string (template for message endpoints with {token} placeholder)
- timeout: number (request timeout in milliseconds)

## Relationships

- ChatSession has many ChatMessage (one-to-many)
- ChatMessage has many SourceReference (one-to-many)
- Single ChatUIState per user session
- APIConfig is global configuration

## Validation Rules

- ChatMessage content must not be empty for user messages
- ChatSession token must be provided when sending messages
- SourceReference URLs must be valid
- ChatMessage timestamp must be in the past or present