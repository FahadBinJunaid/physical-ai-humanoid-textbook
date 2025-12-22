from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID
from datetime import datetime
from enum import Enum


class RoleEnum(str, Enum):
    user = "user"
    assistant = "assistant"


class ChatSessionBase(BaseModel):
    user_id: Optional[UUID] = None
    session_token: str
    metadata: Optional[Dict[str, Any]] = None


class ChatSessionCreate(ChatSessionBase):
    pass


class ChatSession(ChatSessionBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ConversationMessageBase(BaseModel):
    session_id: UUID
    role: RoleEnum
    content: str
    sources: Optional[List[Dict[str, Any]]] = None
    metadata: Optional[Dict[str, Any]] = None


class ConversationMessageCreate(ConversationMessageBase):
    pass


class ConversationMessage(ConversationMessageBase):
    id: UUID
    timestamp: datetime

    class Config:
        from_attributes = True


class KnowledgeChunkBase(BaseModel):
    source_document: str
    chunk_text: str
    chunk_index: int
    document_title: str
    page_number: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class KnowledgeChunkCreate(KnowledgeChunkBase):
    pass


class KnowledgeChunk(KnowledgeChunkBase):
    id: UUID
    created_at: datetime

    class Config:
        from_attributes = True


# API Request/Response Schemas
class ChatStartRequest(BaseModel):
    pass


class ChatStartResponse(BaseModel):
    session_token: str


class ChatMessageRequest(BaseModel):
    message: str


class SourceDocument(BaseModel):
    document: str
    title: str
    content: str
    score: float


class ChatMessageResponse(BaseModel):
    response: str
    sources: List[SourceDocument]
    timestamp: datetime


class ChatHistoryResponse(BaseModel):
    messages: List[ConversationMessage]
    created_at: datetime


class IngestRequest(BaseModel):
    force: Optional[bool] = False


class IngestResponse(BaseModel):
    processed_files: List[str]
    status: str
    message: Optional[str] = None


class HealthResponse(BaseModel):
    status: str
    timestamp: datetime