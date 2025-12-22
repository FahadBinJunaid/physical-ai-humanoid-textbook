import React from 'react';
import { MessageCircle } from 'lucide-react';
import styles from './styles.module.css';

interface FloatingActionButtonProps {
  onClick: () => void;
  isOpen: boolean;
  unreadCount?: number;
}

const FloatingActionButton: React.FC<FloatingActionButtonProps> = ({
  onClick,
  isOpen,
  unreadCount = 0
}) => {
  const handleKeyDown = (e: React.KeyboardEvent<HTMLButtonElement>) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      onClick();
    }
  };

  return (
    <button
      className={`${styles['fab-button']} ${isOpen ? styles.hidden : ''}`}
      onClick={onClick}
      onKeyDown={handleKeyDown}
      aria-label={isOpen ? "Close chat" : "Open chat"}
      tabIndex={0}
    >
      <MessageCircle size={24} />
      {unreadCount > 0 && (
        <span className={styles['unread-badge']}>
          {unreadCount}
        </span>
      )}
    </button>
  );
};

export default FloatingActionButton;