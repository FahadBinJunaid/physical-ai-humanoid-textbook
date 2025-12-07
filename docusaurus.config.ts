import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  // --- Branding Fixes ---
  title: 'Physical AI Textbook', 
  tagline: 'Bridging the gap between Digital Intelligence and the Physical World', 
  favicon: 'img/favicon.ico',

  // --- Project Configuration ---
  url: 'https://FahadBinJunaid.github.io', 
  baseUrl: '/', 
  
  organizationName: 'FahadBinJunaid', 
  projectName: 'Physical AI Textbook', 
  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts', 
          routeBasePath: '/', // Makes the docs the homepage
        },
        // Blog is REMOVED
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark', 
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook', 
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg', 
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook Modules',
        },
        {
          href: 'https://github.com/FahadBinJunaid', // Fixed GitHub link
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course Modules',
          items: [
            // Links MUST match the correct Docusaurus IDs from sidebars.ts
            { label: 'Start Learning (Intro)', to: '/intro/welcome' },
            { label: 'Module 1: ROS 2', to: '/module-1-ros2/nodes-intro' },
            { label: 'Module 2: Simulation', to: '/module-2-simulation/gazebo-physics' },
            { label: 'Module 3: NVIDIA Isaac', to: '/module-3-isaac/isaac-sim-intro' },
            { label: 'Module 4: VLA', to: '/module-4-vla/whisper-voice' },
          ],
        },
        {
          title: 'Project Links',
          items: [
            // Fixed GitHub link
            { label: 'GitHub Profile', href: 'https://github.com/FahadBinJunaid' }, 
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Fahad Bin Junaid. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;