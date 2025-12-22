import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useColorMode} from '@docusaurus/theme-common';
import styles from './ModuleCard.module.css';

interface ModuleCardProps {
  title: string;
  description?: string;
  link: string;
  icon?: string;
  color?: string;
}

const ModuleCard: React.FC<ModuleCardProps> = ({
  title,
  description,
  link,
  icon = '📚',
  color = 'blue'
}) => {
  const {colorMode} = useColorMode();
  const isDark = colorMode === 'dark';

  // Define color classes based on the color prop
  const colorClasses = {
    blue: {
      light: 'module-card-blue-light',
      dark: 'module-card-blue-dark'
    },
    slate: {
      light: 'module-card-slate-light',
      dark: 'module-card-slate-dark'
    },
    teal: {
      light: 'module-card-teal-light',
      dark: 'module-card-teal-dark'
    },
    indigo: {
      light: 'module-card-indigo-light',
      dark: 'module-card-indigo-dark'
    }
  };

  const cardColor = colorClasses[color as keyof typeof colorClasses] || colorClasses.blue;
  const themeClass = isDark ? cardColor.dark : cardColor.light;

  return (
    <Link
      to={link}
      className={clsx('card', styles.moduleCard, styles[themeClass])}
      style={{ textDecoration: 'none' }}
    >
      <div className={styles.cardHeader}>
        <span className={styles.cardIcon}>{icon}</span>
        <h3 className={styles.cardTitle}>{title}</h3>
      </div>
      {description && (
        <p className={styles.cardDescription}>{description}</p>
      )}
      <div className={styles.cardFooter}>
        <span className={styles.learnMoreText}>Start Learning →</span>
      </div>
    </Link>
  );
};

export default ModuleCard;