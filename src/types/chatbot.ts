// Type definitions for Chatbot UI

export interface ChatSession {
  id: string; // unique session identifier from backend
  token: string; // session token for API communication
  createdAt: Date; // when session was initiated
  isActive: boolean; // whether session is currently active
}

export interface SourceReference {
  id: string; // unique identifier for the source
  title: string; // display title for the source
  url: string; // URL to the original documentation
  page: string; // page or section name
  relevance?: number; // optional relevance score from 0-1
}

export interface ChatMessage {
  id: string; // unique message identifier
  sessionId: string; // reference to parent session
  sender: 'user' | 'bot'; // who sent the message
  content: string; // the message content
  timestamp: Date; // when message was created
  sources?: SourceReference[]; // optional source references for bot responses
  status: 'pending' | 'sent' | 'delivered' | 'error'; // message status for UI
}

export interface ChatUIState {
  isOpen: boolean; // whether chat window is visible
  isMinimized: boolean; // whether chat window is minimized
  position: { x: number; y: number }; // coordinates for window positioning
  unreadCount: number; // number of unread messages
  lastActive: Date; // when chat was last interacted with
}

export interface APIConfig {
  baseUrl: string; // base URL for API endpoints
  startEndpoint: string; // endpoint for starting new sessions
  messageEndpoint: string; // template for message endpoints with {token} placeholder
  timeout: number; // request timeout in milliseconds
}