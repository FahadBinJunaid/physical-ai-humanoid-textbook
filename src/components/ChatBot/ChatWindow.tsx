import React, { useState, useEffect, useRef } from 'react';
import { X, Minimize2, Maximize2 } from 'lucide-react';
import MessageBubble from './MessageBubble';
import ChatInput from './ChatInput';
import styles from './styles.module.css';
import { ChatMessage, ChatUIState } from '../../types/chatbot';

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  messages: ChatMessage[];
  onSendMessage: (message: string) => void;
  isLoading?: boolean;
}

const ChatWindow: React.FC<ChatWindowProps> = ({
  isOpen,
  onClose,
  messages,
  onSendMessage,
  isLoading = false
}) => {
  const [minimized, setMinimized] = useState(false);
  const [uiState, setUiState] = useState<ChatUIState>({
    isOpen: true,
    isMinimized: false,
    position: { x: 0, y: 0 },
    unreadCount: 0,
    lastActive: new Date()
  });

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const messagesContainerRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    if (messagesContainerRef.current) {
      // Only auto-scroll if user is near the bottom
      const { scrollTop, scrollHeight, clientHeight } = messagesContainerRef.current;
      const isNearBottom = scrollHeight - scrollTop <= clientHeight + 10;

      if (isNearBottom) {
        scrollToBottom();
      }
    }
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleMinimize = () => {
    setMinimized(!minimized);
    setUiState(prev => ({
      ...prev,
      isMinimized: !prev.isMinimized
    }));
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div
      className={`${styles['chat-window']} ${isOpen ? styles.open : ''} ${minimized ? styles.minimized : ''}`}
      role="dialog"
      aria-modal="true"
      aria-label="AI Robotics Assistant Chat"
      aria-hidden={!isOpen}
    >
      <div className={styles['chat-window-header']} role="banner">
        <div className={styles['chat-window-title-container']}>
          <div className={styles['chat-window-title']} id="chat-window-title">
            AI Robotics Assistant
          </div>
          <div className={styles['chat-window-subtitle']}>
            Powered by Robotics RAG
          </div>
        </div>
        <div className={styles['chat-window-controls']}>
          <button
            onClick={toggleMinimize}
            className={styles['window-control']}
            aria-label={minimized ? "Maximize chat window" : "Minimize chat window"}
            aria-pressed={minimized}
          >
            {minimized ? <Maximize2 size={18} aria-hidden="true" /> : <Minimize2 size={18} aria-hidden="true" />}
          </button>
          <button
            onClick={onClose}
            className={styles['window-control']}
            aria-label="Close chat window"
            aria-describedby="chat-window-title"
          >
            <X size={18} aria-hidden="true" />
          </button>
        </div>
      </div>

      {!minimized && (
        <>
          <div className={styles['chat-window-body']} role="main">
            <div
              className={styles['chat-messages']}
              ref={messagesContainerRef}
              aria-live="polite"
              aria-relevant="additions"
              role="log"
              aria-label="Chat messages"
            >
              {messages.map((message) => (
                <MessageBubble
                  key={message.id}
                  message={message}
                />
              ))}
              {isLoading && (
                <div
                  className={`${styles['message-bubble']} ${styles['bot-message']}`}
                  aria-label="Bot is typing"
                >
                  <div>Thinking...</div>
                </div>
              )}
              <div ref={messagesEndRef} aria-hidden="true" />
            </div>
            <div className={styles['chat-input-area']} role="form" aria-label="Chat input">
              <ChatInput onSendMessage={onSendMessage} disabled={isLoading} />
            </div>
          </div>
        </>
      )}
    </div>
  );
};

export default ChatWindow;