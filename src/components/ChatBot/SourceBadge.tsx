import React from 'react';
import { ExternalLink } from 'lucide-react';
import styles from './styles.module.css';
import { SourceReference } from '../../types/chatbot';

interface SourceBadgeProps {
  source: SourceReference;
}

const SourceBadge: React.FC<SourceBadgeProps> = ({ source }) => {
  const handleClick = () => {
    // Open the source URL in a new tab
    window.open(source.url, '_blank', 'noopener,noreferrer');
  };

  return (
    <button
      className={styles['source-badge']}
      onClick={handleClick}
      title={`Source: ${source.title}`}
      aria-label={`Source: ${source.title}`}
    >
      <ExternalLink size={12} style={{ marginRight: '4px' }} />
      {source.page || source.title}
    </button>
  );
};

export default SourceBadge;