# Quickstart Guide: Homepage Redesign

## Overview
This guide provides the essential steps to implement the homepage redesign for the Physical AI & Humanoid Robotics educational website.

## Prerequisites
- Node.js 18+ installed
- Docusaurus CLI installed
- Access to the project repository

## Setup
1. Clone the repository:
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

## Implementation Steps

### 1. Update Branding in Docusaurus Config
- Update `docusaurus.config.ts` with the new title and tagline:
  - Title: "Physical AI & Humanoid Robotics"
  - Tagline: "Mastering the Future of Embodied Intelligence"

### 2. Remove Blog Functionality
- Delete the entire `/blog` folder
- Remove any references to blog in `docusaurus.config.ts`

### 3. Redesign Homepage
- Modify `src/pages/index.tsx` to include:
  - Updated branding elements
  - "Start Learning" button linking to "/docs/introduction/intro"
  - 4 interactive module cards for ROS2 Basics, Digital Twin, NVIDIA Isaac, and VLA Intelligence

### 4. Update Styling
- Modify `src/css/custom.css` to implement:
  - High text contrast for both light and dark modes
  - Blue/Slate robotic theme
  - Responsive design for module cards

### 5. Update Navigation
- Modify footer in `docusaurus.config.ts` to:
  - Remove "Blog" and "Community" links
  - Add direct links to all 5 modules
  - Ensure "Tutorial" link points to "/docs/introduction/intro"

## Testing
1. Start the development server:
   ```bash
   npm start
   ```

2. Verify all changes in browser:
   - Check updated branding
   - Test "Start Learning" button navigation
   - Verify module card functionality
   - Test light/dark mode contrast
   - Confirm all footer links work correctly

## Build and Deploy
1. Build the site:
   ```bash
   npm run build
   ```

2. Test the build locally:
   ```bash
   npm run serve
   ```