from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class ChatSession(Base):
    """
    Represents an individual user's conversation session with the chatbot
    """
    __tablename__ = "chat_sessions"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(PG_UUID(as_uuid=True), nullable=True)  # Optional, for registered users
    session_token = Column(String(255), unique=True, nullable=False)  # For anonymous sessions
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)
    metadata_ = Column("metadata", JSON)  # Additional session metadata


class ConversationMessage(Base):
    """
    Individual exchanges between user and system in a conversation
    """
    __tablename__ = "conversation_messages"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    sources = Column(JSON)  # Source documents/references used in the response
    metadata_ = Column("metadata", JSON)  # Additional message metadata


class KnowledgeChunk(Base):
    """
    Segments of documentation content processed and stored for retrieval
    (metadata stored in database, actual vectors in Qdrant)
    """
    __tablename__ = "knowledge_chunks"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)  # Matches Qdrant point ID
    source_document = Column(String(500), nullable=False)  # Path to the original MDX file
    chunk_text = Column(Text, nullable=False)  # The actual content of the chunk
    chunk_index = Column(Integer, nullable=False)  # Sequential index of the chunk in the original document
    document_title = Column(String(200), nullable=False)  # Title of the source document
    page_number = Column(Integer, nullable=True)  # Page number if applicable
    metadata_ = Column("metadata", JSON)  # Additional chunk metadata (tags, categories, etc.)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)