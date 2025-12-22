import React from 'react';
import styles from './styles.module.css';

interface LoadingSkeletonProps {
  type?: 'message' | 'full' | 'avatar';
  count?: number;
}

const LoadingSkeleton: React.FC<LoadingSkeletonProps> = ({
  type = 'message',
  count = 1
}) => {
  const skeletons = Array.from({ length: count }, (_, index) => (
    <div
      key={index}
      className={`${styles['loading-skeleton']} ${styles['skeleton-message']} ${type === 'message' ? styles['skeleton-bot-message'] : ''}`}
    />
  ));

  return <>{skeletons}</>;
};

export default LoadingSkeleton;