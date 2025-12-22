import React, { useState, KeyboardEvent } from 'react';
import { Send } from 'lucide-react';
import styles from './styles.module.css';

interface ChatInputProps {
  onSendMessage: (message: string) => void;
  disabled?: boolean;
}

const ChatInput: React.FC<ChatInputProps> = ({ onSendMessage, disabled = false }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = () => {
    if (inputValue.trim() && !disabled) {
      onSendMessage(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div className={styles['chat-input-container']}>
      <textarea
        className={styles['chat-input']}
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder="Type your message here..."
        disabled={disabled}
        rows={1}
      />
      <button
        className={styles['send-button']}
        onClick={handleSubmit}
        disabled={disabled || !inputValue.trim()}
        aria-label="Send message"
      >
        <Send size={20} />
      </button>
    </div>
  );
};

export default ChatInput;