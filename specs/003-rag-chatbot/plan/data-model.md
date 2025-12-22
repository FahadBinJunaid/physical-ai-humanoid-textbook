# Data Model: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration
**Date**: 2025-12-20
**Model Version**: 1.0

## Entity: ChatSession

**Description**: Represents an individual user's conversation session with the chatbot

**Fields**:
- `id`: UUID (Primary Key)
  - Type: UUID
  - Required: Yes
  - Unique: Yes
  - Description: Unique identifier for the session
- `user_id`: UUID (Optional)
  - Type: UUID
  - Required: No
  - Description: Reference to authenticated user (null for anonymous sessions)
- `session_token`: String
  - Type: String(255)
  - Required: Yes
  - Unique: Yes
  - Description: Anonymous session identifier
- `created_at`: DateTime
  - Type: DateTime
  - Required: Yes
  - Default: Current timestamp
  - Description: When the session was created
- `updated_at`: DateTime
  - Type: DateTime
  - Required: Yes
  - Description: When the session was last updated
- `metadata`: JSON
  - Type: JSON
  - Required: No
  - Description: Additional session metadata (user preferences, analytics, etc.)

**Relationships**:
- One ChatSession has many ConversationMessages (via session_id foreign key)

**Validation Rules**:
- session_token must be unique
- created_at must be before or equal to updated_at
- user_id must reference a valid user if provided

## Entity: ConversationMessage

**Description**: Individual exchanges between user and system in a conversation

**Fields**:
- `id`: UUID (Primary Key)
  - Type: UUID
  - Required: Yes
  - Unique: Yes
  - Description: Unique identifier for the message
- `session_id`: UUID
  - Type: UUID
  - Required: Yes
  - Foreign Key: References ChatSession.id
  - Description: Reference to the parent session
- `role`: String
  - Type: String(20)
  - Required: Yes
  - Enum: ['user', 'assistant']
  - Description: Role of the message sender
- `content`: Text
  - Type: Text
  - Required: Yes
  - Description: The actual message content
- `timestamp`: DateTime
  - Type: DateTime
  - Required: Yes
  - Default: Current timestamp
  - Description: When the message was created
- `sources`: JSON
  - Type: JSON
  - Required: No
  - Description: Source documents/references used in the response
- `metadata`: JSON
  - Type: JSON
  - Required: No
  - Description: Additional message metadata

**Relationships**:
- Many ConversationMessages belong to one ChatSession (via session_id foreign key)

**Validation Rules**:
- session_id must reference a valid ChatSession
- role must be either 'user' or 'assistant'
- content must not be empty
- timestamp must be reasonable (not in the future)

## Entity: KnowledgeChunk

**Description**: Segments of documentation content processed and stored for retrieval (metadata stored in database, actual vectors in Qdrant)

**Fields**:
- `id`: UUID
  - Type: UUID
  - Required: Yes
  - Unique: Yes
  - Description: Matches the Qdrant point ID for correlation
- `source_document`: String
  - Type: String(500)
  - Required: Yes
  - Description: Path to the original MDX file
- `chunk_text`: Text
  - Type: Text
  - Required: Yes
  - Description: The actual content of the chunk
- `chunk_index`: Integer
  - Type: Integer
  - Required: Yes
  - Description: Sequential index of the chunk in the original document
- `document_title`: String
  - Type: String(200)
  - Required: Yes
  - Description: Title of the source document
- `page_number`: Integer (Optional)
  - Type: Integer
  - Required: No
  - Description: Page number if applicable
- `metadata`: JSON
  - Type: JSON
  - Required: No
  - Description: Additional chunk metadata (tags, categories, etc.)
- `created_at`: DateTime
  - Type: DateTime
  - Required: Yes
  - Default: Current timestamp
  - Description: When the chunk was created

**Relationships**:
- No direct relationships to other entities (connected via Qdrant)

**Validation Rules**:
- chunk_text must not be empty
- chunk_index must be non-negative
- source_document must be a valid path

## State Transitions

### ChatSession States
- **Active**: Session is currently in use
- **Inactive**: Session has not been updated recently
- **Expired**: Session has exceeded time-to-live

### Transition Rules
- A ChatSession starts as Active when created
- A ChatSession becomes Inactive after a period of inactivity
- An Inactive ChatSession becomes Expired after the session timeout period
- Expired sessions may be cleaned up periodically

## Indexes

### ChatSession
- Index on session_token for fast lookups
- Index on user_id for user-specific queries
- Composite index on (user_id, created_at) for session history

### ConversationMessage
- Index on session_id for session-based queries
- Composite index on (session_id, timestamp) for chronological ordering
- Index on timestamp for time-based queries

### KnowledgeChunk
- Index on source_document for document-based queries
- Index on document_title for search optimization
- Index on created_at for content freshness queries

## Constraints

1. **Referential Integrity**: All foreign key relationships must reference valid records
2. **Session Uniqueness**: Each session_token must be unique across all sessions
3. **Message Order**: Messages within a session should maintain chronological order based on timestamp
4. **Content Validation**: All text fields should be validated for appropriate content
5. **Size Limits**: Large text fields should have appropriate size limits to prevent performance issues