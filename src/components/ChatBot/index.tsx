import React, { useState, useEffect } from 'react';
import { startNewSession, sendMessage, ChatResponse } from '../../services/chatbot-api';
import { ChatSession, ChatMessage } from '../../types/chatbot';
import ChatWindow from './ChatWindow';
import FloatingActionButton from './FloatingActionButton';
import LoadingSkeleton from './LoadingSkeleton';

const SESSION_STORAGE_KEY = 'chatbot-session';
const MESSAGES_STORAGE_KEY = 'chatbot-messages';

/**
 * Main ChatBot component that provides a RAG (Retrieval-Augmented Generation) chat interface
 * integrated globally across the Docusaurus site via Root.js.
 *
 * Features:
 * - Floating action button for easy access
 * - Session management with persistence across page navigation
 * - Real-time chat with message history
 * - Source reference display for bot responses
 * - Responsive design for all device sizes
 * - Accessibility features for screen readers
 * - Error handling and timeout management
 *
 * The component manages its own state for:
 * - Chat session (token, status)
 * - Message history (with 50-message limit)
 * - UI state (open/closed, loading)
 * - Error states
 */
const ChatBot: React.FC = () => {
  const [chatSession, setChatSession] = useState<ChatSession | null>(null);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Initialize chat session on component mount
  useEffect(() => {
    const initializeSession = async () => {
      try {
        setIsLoading(true);

        // Try to restore session from storage
        const storedSession = localStorage.getItem(SESSION_STORAGE_KEY);
        const storedMessages = localStorage.getItem(MESSAGES_STORAGE_KEY);

        if (storedSession) {
          try {
            const sessionData = JSON.parse(storedSession) as ChatSession;
            // Validate session by checking if it's still active (within reasonable time)
            const sessionAge = Date.now() - new Date(sessionData.createdAt).getTime();
            const maxSessionAge = 24 * 60 * 60 * 1000; // 24 hours

            if (sessionAge < maxSessionAge) {
              setChatSession(sessionData);
              if (storedMessages) {
                setMessages(JSON.parse(storedMessages));
              }
            } else {
              // Session too old, create a new one
              localStorage.removeItem(SESSION_STORAGE_KEY);
              localStorage.removeItem(MESSAGES_STORAGE_KEY);
              const session = await startNewSession();
              setChatSession(session);
            }
          } catch (parseErr) {
            // If parsing fails, create a new session
            const session = await startNewSession();
            setChatSession(session);
          }
        } else {
          // No stored session, create a new one
          const session = await startNewSession();
          setChatSession(session);
        }

        setError(null);
      } catch (err) {
        console.error('Failed to initialize chat session:', err);
        setError(err instanceof Error ? err.message : 'Failed to initialize chat session');
      } finally {
        setIsLoading(false);
      }
    };

    initializeSession();
  }, []);

  // Save session and messages to storage when they change
  useEffect(() => {
    if (chatSession) {
      localStorage.setItem(SESSION_STORAGE_KEY, JSON.stringify(chatSession));
    }
  }, [chatSession]);

  useEffect(() => {
    localStorage.setItem(MESSAGES_STORAGE_KEY, JSON.stringify(messages));
  }, [messages]);

  // Add keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Close chat with Escape key
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen]);

  const handleSendMessage = async (messageText: string) => {
    if (!chatSession || !messageText.trim()) return;

    try {
      // Add user message to the chat
      const userMessage: ChatMessage = {
        id: Date.now().toString(),
        sessionId: chatSession.token,
        sender: 'user',
        content: messageText,
        timestamp: new Date(),
        status: 'sent'
      };

      setMessages(prev => {
        const updated = [...prev, userMessage];
        // Limit to 50 messages to manage history
        return updated.length > 50 ? updated.slice(-50) : updated;
      });
      setIsLoading(true);

      // Send message to backend
      const response: ChatResponse = await sendMessage(chatSession.token, messageText);

      // Add bot response to the chat
      setMessages(prev => {
        const updated = [...prev, response.message];
        // Limit to 50 messages to manage history
        return updated.length > 50 ? updated.slice(-50) : updated;
      });
    } catch (err) {
      console.error('Failed to send message:', err);

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: `error-${Date.now()}`,
        sessionId: chatSession.token,
        sender: 'bot',
        content: `Error: ${err instanceof Error ? err.message : 'Failed to send message'}`,
        timestamp: new Date(),
        status: 'error'
      };

      setMessages(prev => {
        const updated = [...prev, errorMessage];
        // Limit to 50 messages to manage history
        return updated.length > 50 ? updated.slice(-50) : updated;
      });
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChatWindow = () => {
    if (!chatSession) {
      // If no session exists, try to create one
      if (!isLoading) {
        const initializeSession = async () => {
          try {
            setIsLoading(true);
            const session = await startNewSession();
            setChatSession(session);
            setIsOpen(true);
            setError(null);
          } catch (err) {
            console.error('Failed to initialize chat session:', err);
            setError(err instanceof Error ? err.message : 'Failed to initialize chat session');
          } finally {
            setIsLoading(false);
          }
        };

        initializeSession();
      }
    } else {
      setIsOpen(!isOpen);
    }
  };

  const handleCloseChat = () => {
    setIsOpen(false);
  };

  return (
    <>
      <FloatingActionButton
        onClick={toggleChatWindow}
        isOpen={isOpen}
      />

      <ChatWindow
        isOpen={isOpen}
        onClose={handleCloseChat}
        messages={messages}
        onSendMessage={handleSendMessage}
        isLoading={isLoading}
      />

      {error && (
        <div style={{ position: 'fixed', bottom: '80px', right: '20px', color: 'red', zIndex: 1001 }}>
          {error}
        </div>
      )}
    </>
  );
};

export default ChatBot;