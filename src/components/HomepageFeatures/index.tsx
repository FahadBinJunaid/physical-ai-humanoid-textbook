import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import ModuleCard from '../ModuleCard';

type ModuleItem = {
  title: string;
  description: string;
  link: string;
  icon: string;
  color: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Introduction',
    description: 'Getting started with Physical AI & Humanoid Robotics',
    link: '/docs/01-introduction/intro',
    icon: '👋',
    color: 'blue',
  },
  {
    title: 'Module 1 - ROS2',
    description: 'Learn the fundamentals of Robot Operating System 2',
    link: '/docs/02-module-1-ros2/ros2-basics',
    icon: '🤖',
    color: 'teal',
  },
  {
    title: 'Module 2 - Digital Twin',
    description: 'Explore creating and managing digital replicas of physical robotic systems',
    link: '/docs/03-module-2-digital-twin/simulation',
    icon: '🔄',
    color: 'slate',
  },
  {
    title: 'Module 3 - NVIDIA Isaac',
    description: 'Master NVIDIA Isaac for developing AI-powered robotic applications',
    link: '/docs/04-module-3-nvidia-isaac/perception',
    icon: '⚡',
    color: 'indigo',
  },
  {
    title: 'Module 4 - VLA Intelligence',
    description: 'Discover Vision-Language-Action models for intelligent robotic systems',
    link: '/docs/05-module-4-vla/vla-intelligence',
    icon: '🧠',
    color: 'blue',
  },
];

function ModuleFeature({title, description, link, icon, color}: ModuleItem) {
  return (
    <div className={clsx('col col--3')}>
      <ModuleCard
        title={title}
        description={description}
        link={link}
        icon={icon}
        color={color}
      />
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row" style={{gap: '1rem', justifyContent: 'center'}}>
          {ModuleList.map((props, idx) => (
            <ModuleFeature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}