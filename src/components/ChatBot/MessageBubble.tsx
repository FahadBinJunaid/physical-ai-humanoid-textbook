import React from 'react';
import ReactMarkdown from 'react-markdown';
import rehypeHighlight from 'rehype-highlight';
import { Clock, Check, CheckCheck, AlertCircle } from 'lucide-react';
import SourceBadge from './SourceBadge';
import styles from './styles.module.css';
import { ChatMessage } from '../../types/chatbot';

interface MessageBubbleProps {
  message: ChatMessage;
}

const MessageBubble: React.FC<MessageBubbleProps> = ({ message }) => {
  const isUser = message.sender === 'user';
  const messageClass = isUser
    ? `${styles['message-bubble']} ${styles['user-message']}`
    : `${styles['message-bubble']} ${styles['bot-message']}`;

  // Determine status icon based on message status
  const getStatusIcon = () => {
    switch (message.status) {
      case 'pending':
        return <Clock size={14} />;
      case 'sent':
        return <Check size={14} />;
      case 'delivered':
        return <CheckCheck size={14} />;
      case 'error':
        return <AlertCircle size={14} color="var(--error-red)" />;
      default:
        return null;
    }
  };

  // Format timestamp to show time only (e.g., "10:30 AM")
  const formatTime = (date: Date) => {
    return new Date(date).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={messageClass}>
      {isUser ? (
        <div>{message.content}</div>
      ) : (
        <ReactMarkdown
          children={message.content}
          rehypePlugins={[rehypeHighlight]}
          components={{
            // Custom rendering for various markdown elements
            p: ({node, ...props}) => <p className={styles['markdown-paragraph']} {...props} />,
            strong: ({node, ...props}) => <strong className={styles['markdown-bold']} {...props} />,
            em: ({node, ...props}) => <em className={styles['markdown-italic']} {...props} />,
            ul: ({node, ...props}) => <ul className={styles['markdown-list']} {...props} />,
            ol: ({node, ...props}) => <ol className={styles['markdown-ordered-list']} {...props} />,
            li: ({node, ...props}) => <li className={styles['markdown-list-item']} {...props} />,
            h1: ({node, ...props}) => <h1 className={styles['markdown-h1']} {...props} />,
            h2: ({node, ...props}) => <h2 className={styles['markdown-h2']} {...props} />,
            h3: ({node, ...props}) => <h3 className={styles['markdown-h3']} {...props} />,
            blockquote: ({node, ...props}) => <blockquote className={styles['markdown-blockquote']} {...props} />,
            code({node, inline, className, children, ...props}) {
              const match = /language-(\w+)/.exec(className || '');
              return !inline && match ? (
                <pre className={styles['code-block']} {...props}>
                  <code className={match[0]}>{children}</code>
                </pre>
              ) : (
                <code className={styles['inline-code']} {...props}>{children}</code>
              );
            }
          }}
        />
      )}

      {/* Display source references for bot messages */}
      {message.sources && message.sources.length > 0 && !isUser && (
        <div className={styles['source-references']}>
          {message.sources.map((source) => (
            <SourceBadge key={source.id} source={source} />
          ))}
        </div>
      )}

      <div className={styles['message-info']}>
        <span className={styles['message-time']}>{formatTime(message.timestamp)}</span>
        <span className={styles['message-status-icon']}>
          {getStatusIcon()}
        </span>
      </div>
    </div>
  );
};

export default MessageBubble;