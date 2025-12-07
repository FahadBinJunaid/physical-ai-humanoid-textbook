import React, { JSX } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';

// --- Styling Definitions (Injected directly to ensure display) ---
const containerStyle: React.CSSProperties = {
  backgroundColor: 'var(--ifm-background-color)', 
  minHeight: '100vh',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  textAlign: 'center',
  padding: '4rem 2rem',
};

const heroContentStyle: React.CSSProperties = {
  maxWidth: '900px',
};

const titleStyle: React.CSSProperties = {
  fontSize: '4.5rem',
  fontWeight: '900',
  color: 'var(--ifm-color-primary)',
  textShadow: '0 0 15px rgba(77, 158, 255, 0.8)', 
  marginBottom: '1rem',
  lineHeight: '1.1',
};

const taglineStyle: React.CSSProperties = {
  fontSize: '1.5rem',
  color: 'var(--ifm-color-emphasis-100)',
  marginBottom: '2rem',
};

const moduleGridStyle: React.CSSProperties = {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '20px',
    marginTop: '3rem',
    marginBottom: '3rem',
};

const moduleCardStyle: React.CSSProperties = {
    backgroundColor: 'var(--ifm-navbar-background-color)',
    padding: '20px',
    borderRadius: '8px',
    border: '1px solid var(--ifm-border-color)',
    textAlign: 'left',
    transition: 'transform 0.3s ease, box-shadow 0.3s ease',
    boxShadow: '0 0 5px rgba(36, 227, 132, 0.1)',
    height: '100%',
};

// --- Module Data (Paths match the corrected sidebars.ts file) ---
const modules = [
  { title: "Module 1: ROS 2", description: "The Robotic Nervous System", to: "/module-1-ros2/nodes-intro" },
  { title: "Module 2: Digital Twin", description: "Physics Simulation (Gazebo/Unity)", to: "/module-2-simulation/gazebo-physics" },
  { title: "Module 3: AI-Robot Brain", description: "NVIDIA Isaac & VSLAM Acceleration", to: "/module-3-isaac/isaac-sim-intro" },
  { title: "Module 4: VLA", description: "Vision-Language-Action & LLM Planning", to: "/module-4-vla/whisper-voice" },
];


// --- Components ---
const ModuleCard = ({ title, description, to }) => (
    <Link to={to} style={{textDecoration: 'none'}}>
      <div style={moduleCardStyle}>
          <h3 style={{ color: 'var(--ifm-color-secondary)', fontSize: '1.2rem', marginBottom: '0.5rem' }}>{title}</h3>
          <p style={{ color: 'var(--ifm-color-emphasis-100)', opacity: 0.7 }}>{description}</p>
      </div>
    </Link>
);


function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header style={containerStyle}>
      <div style={heroContentStyle}>
        <Heading as="h1" style={titleStyle}>
          {siteConfig.title} 
        </Heading>
        <p style={taglineStyle}>{siteConfig.tagline}</p>
        
        <Link
          className="button button--lg"
          to="/intro/welcome" // Link to the first tutorial page
          style={{ backgroundColor: 'var(--ifm-color-secondary)', color: 'var(--ifm-background-color)', fontWeight: 'bold' }}>
          START YOUR JOURNEY 🚀
        </Link>
        
        <div style={moduleGridStyle}>
          {modules.map((module, index) => (
            <ModuleCard key={index} {...module} />
          ))}
        </div>
        
        <p style={{ color: 'var(--ifm-color-primary)', fontSize: '1rem' }}>
          Fully compliant with the Agent Hackathon specification.
        </p>

      </div>
    </header>
  );
}


export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A textbook for teaching Physical AI & Humanoid Robotics">
      <HomepageHeader />
    </Layout>
  );
}