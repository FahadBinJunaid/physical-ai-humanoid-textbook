# API Contract: RAG Chatbot Backend Integration

**Feature**: 004-rag-chatbot-ui
**Date**: 2025-12-21
**Status**: Complete

## Endpoints

### POST /chat/start
**Description**: Initialize a new chat session with the RAG system

**Request**:
- Method: POST
- URL: http://127.0.0.1:8000/chat/start
- Headers:
  - Content-Type: application/json
- Body: {}
- Authentication: None required

**Response**:
- Success: 200 OK
- Body:
```json
{
  "token": "string",
  "session_id": "string",
  "message": "Session started successfully"
}
```

**Error Responses**:
- 500 Internal Server Error: Backend service unavailable

### POST /chat/{token}/message
**Description**: Send a message to the RAG system and receive a response

**Request**:
- Method: POST
- URL: http://127.0.0.1:8000/chat/{token}/message
- Headers:
  - Content-Type: application/json
- Path Parameters:
  - token: string (session token from /chat/start)
- Body:
```json
{
  "message": "string"
}
```
- Authentication: None required

**Response**:
- Success: 200 OK
- Body:
```json
{
  "response": "string",
  "sources": [
    {
      "id": "string",
      "title": "string",
      "url": "string",
      "page": "string",
      "relevance": 0.0
    }
  ]
}
```

**Error Responses**:
- 404 Not Found: Invalid session token
- 500 Internal Server Error: Backend service unavailable

## Data Types

### ChatResponse
- response: string (the bot's response to the user's message)
- sources: Source[] (list of sources used to generate the response)

### Source
- id: string (unique identifier for the source)
- title: string (display title for the source)
- url: string (URL to the original documentation)
- page: string (page or section name)
- relevance: number (relevance score from 0.0 to 1.0)

## Error Handling

All API calls should implement proper error handling with user-friendly messages displayed in the UI when:
- Network requests fail
- Backend service is unavailable
- Invalid session tokens are provided
- Request timeouts occur