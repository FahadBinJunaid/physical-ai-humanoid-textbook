"""
Conversation State Management for Multi-Turn Interactions

This module provides a comprehensive system for tracking conversation state
across multiple interactions, including context management, state persistence,
and conversation flow control.
"""
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from datetime import datetime
import json
import uuid


@dataclass
class Message:
    """Represents a single message in the conversation"""
    id: str
    role: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ConversationState:
    """Represents the state of a single conversation session"""
    session_id: str
    created_at: datetime
    messages: List[Message] = field(default_factory=list)
    context_summary: str = ""
    turn_count: int = 0
    active: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)

    def add_message(self, role: str, content: str, metadata: Optional[Dict[str, Any]] = None) -> Message:
        """Add a message to the conversation"""
        message = Message(
            id=str(uuid.uuid4()),
            role=role,
            content=content,
            timestamp=datetime.now(),
            metadata=metadata or {}
        )
        self.messages.append(message)
        self.turn_count += 1
        return message

    def get_recent_messages(self, count: int = 5) -> List[Message]:
        """Get the most recent messages from the conversation"""
        return self.messages[-count:] if len(self.messages) >= count else self.messages[:]

    def get_messages_as_dicts(self) -> List[Dict[str, Any]]:
        """Convert messages to dictionary format for the agent"""
        return [
            {
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat() if msg.timestamp else None,
                "metadata": msg.metadata
            }
            for msg in self.messages
        ]


class ConversationStateManager:
    """
    Manages conversation states across multiple sessions
    """
    def __init__(self):
        self.conversations: Dict[str, ConversationState] = {}

    def create_conversation(self, session_id: Optional[str] = None) -> ConversationState:
        """Create a new conversation session"""
        session_id = session_id or str(uuid.uuid4())

        conversation = ConversationState(
            session_id=session_id,
            created_at=datetime.now()
        )

        self.conversations[session_id] = conversation
        return conversation

    def get_conversation(self, session_id: str) -> Optional[ConversationState]:
        """Get an existing conversation by session ID"""
        return self.conversations.get(session_id)

    def update_conversation_context(self, session_id: str, context_summary: str) -> bool:
        """Update the context summary for a conversation"""
        conversation = self.get_conversation(session_id)
        if conversation:
            conversation.context_summary = context_summary
            return True
        return False

    def add_message_to_conversation(self, session_id: str, role: str, content: str,
                                  metadata: Optional[Dict[str, Any]] = None) -> Optional[Message]:
        """Add a message to a conversation"""
        conversation = self.get_conversation(session_id)
        if conversation:
            return conversation.add_message(role, content, metadata)
        return None

    def get_conversation_history(self, session_id: str, limit: Optional[int] = None) -> Optional[List[Dict[str, Any]]]:
        """Get conversation history as dictionaries"""
        conversation = self.get_conversation(session_id)
        if conversation:
            messages = conversation.get_messages_as_dicts()
            if limit:
                return messages[-limit:]
            return messages
        return None

    def end_conversation(self, session_id: str) -> bool:
        """Mark a conversation as inactive"""
        conversation = self.get_conversation(session_id)
        if conversation:
            conversation.active = False
            return True
        return False

    def cleanup_inactive_conversations(self, max_age_minutes: int = 30) -> int:
        """Remove conversations that have been inactive for too long"""
        cutoff_time = datetime.now()
        inactive_to_remove = []

        for session_id, conversation in self.conversations.items():
            if not conversation.active:
                # Check if conversation is too old
                if (cutoff_time - conversation.created_at).total_seconds() > (max_age_minutes * 60):
                    inactive_to_remove.append(session_id)

        for session_id in inactive_to_remove:
            del self.conversations[session_id]

        return len(inactive_to_remove)


# Global instance for the application
conversation_manager = ConversationStateManager()


def get_conversation_state_manager() -> ConversationStateManager:
    """Get the global conversation state manager instance"""
    return conversation_manager