---
title: Robotics Handbook RAG Chatbot
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
app_file: main.py
pinned: false
license: mit
---

# Robotics Handbook RAG Chatbot

This is a RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about robotics concepts and receive answers grounded in the documentation.

## Features

- Natural language Q&A interface for robotics documentation
- Context-aware conversation management
- Source citation for all responses
- Session-based conversation history
- Fast response times with vector similarity search

## Architecture

The backend uses:
- **FastAPI**: For the web framework and API endpoints
- **Qdrant Cloud**: For vector storage and similarity search
- **Cohere**: For document embeddings
- **OpenRouter**: For response generation
- **PostgreSQL**: For conversation history storage

## API Endpoints

- `POST /chat/start` - Start a new chat session
- `POST /chat/{session_token}/message` - Send a message to the chatbot
- `GET /chat/{session_token}/history` - Get conversation history
- `GET /health` - Health check endpoint

## Environment Variables

The following environment variables need to be configured for the application to work:

- `QDRANT_URL`: Qdrant Cloud endpoint
- `QDRANT_API_KEY`: Qdrant API key
- `COHERE_API_KEY`: Cohere API key
- `OPENROUTER_API_KEY`: OpenRouter API key
- `DATABASE_URL`: PostgreSQL connection string
- `CHAT_MODEL_NAME`: Name of the LLM model to use
- `EMBEDDING_MODEL_NAME`: Name of the embedding model to use
- `COLLECTION_NAME`: Qdrant collection name for storing embeddings

## Usage

Once deployed, you can interact with the chatbot through the API endpoints. The application will be available on port 7860.

To start a conversation:
1. Call `POST /chat/start` to get a session token
2. Use the session token in `POST /chat/{session_token}/message` to send messages
3. Retrieve conversation history with `GET /chat/{session_token}/history`

## Security

- API keys are stored in environment variables
- Rate limiting is implemented to prevent abuse
- Input validation is performed on all user inputs