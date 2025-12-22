# Quickstart Guide: RAG Chatbot Implementation

**Feature**: RAG Chatbot Integration
**Date**: 2025-12-20
**Version**: 1.0

## Overview

This guide provides step-by-step instructions to set up, configure, and run the RAG Chatbot for the Robotics Handbook. The system uses FastAPI as the backend, Qdrant Cloud for vector storage, Cohere for embeddings, and Google Gemini for response generation.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Git (for version control)
- Access to the following APIs:
  - Qdrant Cloud (endpoint: https://41561210-ab70-4c14-80a1-7d0a1aae50f8.europe-west3-0.gcp.cloud.qdrant.io)
  - Cohere API
  - Google Gemini API
  - Neon PostgreSQL (or compatible PostgreSQL)

## Step 1: Repository Setup

1. Clone the repository (if not already done):
   ```bash
   git clone <repository-url>
   cd my-website
   ```

2. Navigate to the project root where you'll create the chatbot-backend directory:
   ```bash
   pwd  # Should be at the project root
   ```

## Step 2: Create Backend Directory

1. Create the chatbot-backend directory:
   ```bash
   mkdir chatbot-backend
   cd chatbot-backend
   ```

2. Verify the directory structure:
   ```
   my-website/
   ├── chatbot-backend/        # <- You are here
   ├── docs/                   # Existing documentation
   ├── src/                    # Existing frontend code
   └── ... (other files)
   ```

## Step 3: Environment Configuration

1. Create the `.env` file with the following structure:
   ```bash
   # Create .env file
   touch .env
   ```

2. Add the following environment variables to `.env`:
   ```env
   # Qdrant Configuration
   QDRANT_URL=https://41561210-ab70-4c14-80a1-7d0a1aae50f8.europe-west3-0.gcp.cloud.qdrant.io
   QDRANT_API_KEY=your_qdrant_api_key_here

   # Cohere Configuration
   COHERE_API_KEY=your_cohere_api_key_here

   # Google Gemini Configuration
   GEMINI_API_KEY=your_gemini_api_key_here

   # Database Configuration
   DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/your_db_name?sslmode=require

   # Application Configuration
   CHAT_MODEL_NAME=gemini-2.0-flash
   EMBEDDING_MODEL_NAME=embed-english-v3.0
   COLLECTION_NAME=robotics_docs
   CHUNK_SIZE=1000
   CHUNK_OVERLAP=100

   # Security
   SECRET_KEY=your_secret_key_here
   ALGORITHM=HS256
   ACCESS_TOKEN_EXPIRE_MINUTES=30
   ```

3. Create an example environment file:
   ```bash
   cp .env .env.example
   ```

## Step 4: Dependencies Setup

1. Create `requirements.txt`:
   ```txt
   fastapi==0.104.1
   uvicorn[standard]==0.24.0
   python-dotenv==1.0.0
   qdrant-client==1.9.1
   cohere==5.5.8
   google-generativeai==0.4.1
   langchain==0.1.16
   langchain-community==0.0.38
   langchain-core==0.1.52
   sqlalchemy==2.0.23
   psycopg2-binary==2.9.9
   pydantic==2.5.0
   pydantic-settings==2.1.0
   python-multipart==0.0.6
   alembic==1.13.1
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Step 5: Project Structure

Create the following directory structure:
```
chatbot-backend/
├── .env
├── .env.example
├── requirements.txt
├── main.py                 # FastAPI application
├── agent.py                # RAG agent logic
├── ingest.py               # Document ingestion script
├── models.py               # Database models
├── schemas.py              # Pydantic schemas
├── database.py             # Database connection
├── retriever.py            # Qdrant retrieval logic
├── tools.py                # Tool definitions
├── document_processor.py   # MDX processing utilities
└── config.py               # Configuration utilities
```

## Step 6: Initialize the Vector Database

1. Run the ingestion script to process documentation:
   ```bash
   python ingest.py
   ```

2. This will:
   - Read all MDX files from the parent `docs/` directory
   - Split them using Recursive Character Text Splitter (chunk size: 1000, overlap: 100)
   - Generate embeddings using Cohere embed-english-v3.0
   - Store embeddings in Qdrant Cloud

## Step 7: Start the Application

1. Run the FastAPI server:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

2. The API will be available at `http://localhost:8000`

3. API documentation will be available at `http://localhost:8000/docs`

## Step 8: Test the API

1. Start a new chat session:
   ```bash
   curl -X POST http://localhost:8000/chat/start
   ```

2. Send a message (replace SESSION_TOKEN with the token from step 1):
   ```bash
   curl -X POST http://localhost:8000/chat/SESSION_TOKEN/message \
        -H "Content-Type: application/json" \
        -d '{"message": "What is ROS 2? Explain in simple terms."}'
   ```

## Troubleshooting

### Common Issues

1. **API Keys Not Working**:
   - Verify all API keys are correctly set in `.env`
   - Check that the `.env` file is in the correct directory
   - Restart the server after updating environment variables

2. **Qdrant Connection Issues**:
   - Verify the Qdrant URL is correct
   - Check that the QDRANT_API_KEY is valid
   - Ensure network connectivity to the Qdrant endpoint

3. **Document Ingestion Fails**:
   - Verify the `docs/` directory exists in the parent directory
   - Check that MDX files are properly formatted
   - Ensure sufficient API quota for embedding generation

4. **Database Connection Issues**:
   - Verify the DATABASE_URL is correctly formatted
   - Check that Neon PostgreSQL is properly configured
   - Ensure the database credentials are correct

### Verification Commands

1. Check environment variables:
   ```bash
   python -c "from dotenv import load_dotenv; load_dotenv(); import os; print('QDRANT_URL' in os.environ)"
   ```

2. Test database connection:
   ```bash
   python -c "from database import engine; engine.connect(); print('Database connection successful')"
   ```

3. Test Qdrant connection:
   ```bash
   python -c "from qdrant_client import QdrantClient; client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); client.get_collections(); print('Qdrant connection successful')"
   ```

## Next Steps

1. Integrate with the frontend Docusaurus application
2. Add authentication for registered users
3. Implement rate limiting and monitoring
4. Add comprehensive logging and error tracking
5. Set up automated testing