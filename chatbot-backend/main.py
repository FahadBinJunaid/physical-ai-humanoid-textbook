from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from typing import Generator
from sqlalchemy.orm import Session
import uuid
from datetime import datetime
import asyncio

import models, schemas, database
from database import SessionLocal, engine, create_tables
from config import settings
from agent import RAGAgent
from models import ChatSession, ConversationMessage
from performance import monitor, time_limited_execution
from conversation_state import get_conversation_state_manager


# Create tables on startup
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Create tables on startup
    create_tables()
    yield
    # Cleanup on shutdown if needed
    pass


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Robotics Handbook RAG Chatbot Integration",
    version="1.0.0",
    lifespan=lifespan
)


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Dependency to get database session
def get_db() -> Generator[Session, None, None]:
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Basic health check endpoint
@app.get("/health", response_model=schemas.HealthResponse)
async def health_check():
    return schemas.HealthResponse(
        status="healthy",
        timestamp=datetime.now()
    )


# Chat session creation endpoint (T028)
@app.post("/chat/start", response_model=schemas.ChatStartResponse)
async def start_chat():
    """
    Start a new chat session and return a session token
    """
    session_token = str(uuid.uuid4())

    # Create session in database
    db = next(get_db())
    try:
        db_session = ChatSession(
            session_token=session_token,
            metadata_={}
        )
        db.add(db_session)
        db.commit()
        db.refresh(db_session)
    finally:
        db.close()

    # Initialize conversation state tracking (T043)
    rag_agent = RAGAgent()
    rag_agent.create_conversation_session(session_token)

    return schemas.ChatStartResponse(session_token=session_token)


# Chat message endpoint with retrieval and response generation (T029)
@app.post("/chat/{session_token}/message", response_model=schemas.ChatMessageResponse)
@time_limited_execution(timeout=5.0)  # Ensure response within 5 seconds (SC-001)
async def send_message(session_token: str, request: schemas.ChatMessageRequest, db: Session = Depends(get_db)):
    """
    Send a message to the chatbot and receive a response with source citations
    """
    # Verify session exists
    db_session = db.query(ChatSession).filter(ChatSession.session_token == session_token).first()
    if not db_session:
        raise HTTPException(status_code=404, detail="Session not found")

    # Create RAG agent
    rag_agent = RAGAgent()

    # Get conversation history from state manager for context (T043)
    conversation_history = rag_agent.get_conversation_context(session_token, max_messages=5)

    # If no conversation history in state manager, get from database as fallback
    if not conversation_history:
        db_conversation_history = db.query(ConversationMessage).filter(
            ConversationMessage.session_id == db_session.id
        ).order_by(ConversationMessage.timestamp).all()

        # Convert to dictionary format for the agent
        conversation_history = []
        for msg in db_conversation_history:
            conversation_history.append({
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat() if msg.timestamp else None
            })

    # Process the query with context (T036, T043)
    result = rag_agent.chat_with_context(request.message, conversation_history)  # Using context-aware method

    response_text = result.get("response", "")
    sources = result.get("sources", [])

    # Track conversation state for multi-turn interactions (T043)
    rag_agent.track_conversation_state(session_token, request.message, response_text)

    # Save user message to database
    user_message = ConversationMessage(
        session_id=db_session.id,
        role="user",
        content=request.message
    )
    db.add(user_message)

    # Save assistant response to database
    assistant_message = ConversationMessage(
        session_id=db_session.id,
        role="assistant",
        content=response_text,
        sources=[source.dict() for source in sources] if sources else []
    )
    db.add(assistant_message)

    db.commit()

    return schemas.ChatMessageResponse(
        response=response_text,
        sources=sources,
        timestamp=datetime.now()
    )


@app.get("/chat/{session_token}/history", response_model=schemas.ChatHistoryResponse)
async def get_chat_history(session_token: str, db: Session = Depends(get_db)):
    """
    Retrieve the conversation history for a given session
    """
    # Verify session exists
    db_session = db.query(ChatSession).filter(ChatSession.session_token == session_token).first()
    if not db_session:
        raise HTTPException(status_code=404, detail="Session not found")

    # Get all messages for this session
    messages = db.query(ConversationMessage).filter(
        ConversationMessage.session_id == db_session.id
    ).order_by(ConversationMessage.timestamp).all()

    # Convert to schema format
    schema_messages = []
    for msg in messages:
        schema_msg = schemas.ConversationMessage(
            id=msg.id,
            session_id=msg.session_id,
            role=msg.role,
            content=msg.content,
            timestamp=msg.timestamp,
            sources=msg.sources
        )
        schema_messages.append(schema_msg)

    return schemas.ChatHistoryResponse(
        messages=schema_messages,
        created_at=db_session.created_at
    )


@app.post("/ingest/docs", response_model=schemas.IngestResponse)
async def ingest_documents_endpoint(request: schemas.IngestRequest = schemas.IngestRequest()):
    """
    Process and ingest documentation into the vector database
    """
    import asyncio
    from .ingest import ingest_documents

    result = await ingest_documents(force=request.force)
    return schemas.IngestResponse(**result)


if __name__ == "__main__":
    import uvicorn
    # Use port 7860 for Hugging Face compatibility
    uvicorn.run(
        "main:app",
        host=settings.host,
        port=7860,
        reload=True
    )