---
id: 1
title: Create Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

```text
Project: Physical AI & Humanoid Robotics Textbook with Integrated RAG Chatbot

Core Principles:

Technical Precision: All content regarding ROS 2, NVIDIA Isaac, and VLA models must be technically accurate and reflect 2024-2025 industry standards.

Spec-Driven Consistency: Every implementation phase must strictly follow the current sp.specify, sp.plan, and sp.tasks artifacts.

AI-Native Education: The book should not just explain AI; it should demonstrate it through clean code and executable examples.

Source of Truth: The Docusaurus Markdown files are the primary source of truth for both the website and the RAG vector database.

Key Standards:

Documentation: Use Docusaurus v3 features. Content must be written in MDX.

Diagrams: Use Mermaid.js syntax for all robotics architecture and RAG flow diagrams.

Code Quality: Python code must follow PEP 8; FastAPI code must use Pydantic models for request/response validation.

RAG Architecture: Use OpenAI Agents SDK with Qdrant Cloud for vector storage and Neon Postgres for metadata/chat history.

Robotics Stack: References must focus on ROS 2 (Jazzy/Humble), Gazebo Harmonic, and NVIDIA Isaac Sim/ROS.

Constraints:

Environment: Must be deployable to GitHub Pages (Frontend) and Cloud platforms like Vercel/Railway (Backend).

Security: No API keys or database strings in the code; use .env files and secret management.

RAG Scope: The chatbot must prioritize "Contextual Grounding" (only answer based on the textbook content unless instructed otherwise).

File Structure: Maintain a strict separation between /docs (book content) and /src (RAG logic/FastAPI).

Success Criteria:

Unified Project: A fully functional Docusaurus site where users can read about Humanoid Robotics and chat with the content via an embedded UI.

Accuracy: The RAG chatbot successfully retrieves and answers questions from all 4 Modules of the book.

Zero Drift: The code and content never deviate from the agreed-upon Specifications and Plans.
```

## Response snapshot

Created a comprehensive project constitution for the Physical AI & Humanoid Robotics Textbook with Integrated RAG Chatbot. The constitution establishes six core principles: Technical Precision, Spec-Driven Consistency, AI-Native Education, Source of Truth, RAG Contextual Grounding, and Separation of Concerns. It also defines technical standards, constraints, development workflow, and governance procedures. The constitution ensures all implementation follows the specified technology stack (Docusaurus v3, FastAPI, Qdrant Cloud, Neon Postgres) while maintaining educational integrity and security best practices.

## Outcome

- ✅ Impact: Established foundational governance document defining project principles and standards
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Proceed with feature specification using sp.specify
- 🧠 Reflection: Constitution provides clear guidelines for maintaining technical precision and educational quality throughout the project lifecycle

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification phase