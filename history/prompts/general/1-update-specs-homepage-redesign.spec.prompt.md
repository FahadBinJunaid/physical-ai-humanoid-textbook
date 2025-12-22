---
id: 1
title: update-specs-homepage-redesign
stage: spec
date: 2025-12-18
model: claude-sonnet-4-5-20250929
surface: agent
feature: homepage-redesign
branch: 002-homepage-redesign
user: claude-code
command: /sp.specify
labels: ["specification", "homepage", "redesign", "ui"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files: []
tests: []
---

# Update Specifications for Homepage Redesign

## User Input

```
Update the specifications for Phase 2. Task Phase 1 is done, but we must fix the homepage link. Update as follows:

- Branding: Update docusaurus.config.ts - Title: "Physical AI & Humanoid Robotics", Tagline: "Mastering the Future of Embodied Intelligence".

- Cleanup: Delete the /blog folder. Remove all "Blog" and "Community" links from Navbar and Footer.

- Homepage Fix: Redesign src/pages/index.tsx. The "Start Learning" button MUST link to "/docs/introduction/intro" (based on my 01-introduction folder).

- Module Cards: Add 4 interactive cards on the homepage for: 1. ROS2 Basics, 2. Digital Twin, 3. NVIDIA Isaac, 4. VLA Intelligence.

- Visibility: Update src/css/custom.css for high text contrast in Light/Dark modes using Blue/Slate robotic theme.

- Footer: Link the footer "Tutorial" text to "/docs/introduction/intro" and add direct links to all 5 modules.
```

## Response Summary

Created a comprehensive specification for the homepage redesign feature in the file `./specs/002-homepage-redesign/spec.md`. The specification includes:

- Updated branding requirements for title and tagline
- Cleanup requirements to remove blog and community links
- Homepage redesign with "Start Learning" button linking to the correct documentation
- Addition of 4 interactive module cards for different learning topics
- Accessibility improvements with high contrast themes
- Footer updates with direct module links

The specification follows the required template structure with user scenarios, functional requirements, and measurable success criteria. A quality checklist was also created to validate the specification completeness.

## Outcome

The specification is complete and ready for the planning phase (`/sp.plan`). The feature branch `002-homepage-redesign` has been created with all necessary specification artifacts.