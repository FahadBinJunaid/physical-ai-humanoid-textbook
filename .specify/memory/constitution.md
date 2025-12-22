<!--
Sync Impact Report:
Version change: 1.0.0 → 1.0.0 (initial constitution)
Modified principles: None (new project)
Added sections: All sections (initial constitution)
Removed sections: None
Templates requiring updates: N/A (initial constitution)
Follow-up TODOs: [RATIFICATION_DATE]: Need to set initial ratification date
-->

# Physical AI & Humanoid Robotics Textbook with Integrated RAG Chatbot Constitution

## Core Principles

### Technical Precision
All content regarding ROS 2, NVIDIA Isaac, and VLA models must be technically accurate and reflect 2024-2025 industry standards. This ensures the educational material remains current and practically applicable for students and professionals in the field of humanoid robotics.

### Spec-Driven Consistency
Every implementation phase must strictly follow the current sp.specify, sp.plan, and sp.tasks artifacts. This ensures zero drift between planned functionality and implemented features, maintaining alignment between educational content and technical implementation.

### AI-Native Education
The textbook should not just explain AI concepts; it should demonstrate them through clean, executable code examples. This principle emphasizes hands-on learning through practical implementation and experimentation.

### Source of Truth
The Docusaurus Markdown files serve as the primary source of truth for both the website and the RAG vector database. This ensures content consistency across both the educational material and the AI chatbot's knowledge base.

### RAG Contextual Grounding
The integrated chatbot must prioritize "Contextual Grounding," meaning it only answers questions based on the textbook content unless explicitly instructed otherwise. This maintains educational integrity and prevents hallucination of information.

### Separation of Concerns
Maintain a strict separation between /docs (book content) and /src (RAG logic/FastAPI). This architectural principle ensures clean organization and independent maintenance of content versus implementation logic.

## Technical Standards and Constraints

### Documentation Standards
Use Docusaurus v3 features with content written in MDX format. All diagrams must utilize Mermaid.js syntax for robotics architecture and RAG flow diagrams, ensuring consistent visualization of complex concepts.

### Code Quality Requirements
Python code must follow PEP 8 standards; FastAPI code must use Pydantic models for request/response validation. This ensures maintainable, standardized code across the project.

### Technology Stack Requirements
- Frontend: Docusaurus v3 for documentation and educational content
- Backend: FastAPI for RAG services
- Vector Storage: Qdrant Cloud for efficient similarity search
- Metadata Storage: Neon Postgres for chat history and metadata
- Robotics Stack References: Focus on ROS 2 (Jazzy/Humble), Gazebo Harmonic, and NVIDIA Isaac Sim/ROS

### Security and Deployment
No API keys or database connection strings in the codebase; use .env files and proper secret management. The frontend must be deployable to GitHub Pages, while the backend should work on cloud platforms like Vercel or Railway.

## Development Workflow

### Implementation Discipline
Follow Spec-Driven Development practices with strict adherence to the spec-plan-tasks workflow. All changes must be small, testable, and precisely referenced to existing code. Prioritize the smallest viable diff and avoid unrelated refactoring.

### Quality Assurance
Implement comprehensive testing covering both the educational content accuracy and the RAG system functionality. Include unit tests for code examples, integration tests for the chatbot retrieval mechanisms, and validation of content grounding.

### File Structure Governance
Maintain organized project structure with clear separation between educational modules (organized by topic) and technical implementation (organized by function). Ensure consistent naming conventions and folder organization.

## Governance

This constitution establishes the foundational principles that supersede all other development practices for this project. All team members must verify compliance with these principles during code reviews and implementation phases. Any deviation from these principles must be documented and approved through formal amendment procedures.

Amendments to this constitution require explicit documentation of the change, approval from project stakeholders, and a migration plan for existing implementations. All changes must maintain the educational integrity and technical excellence standards outlined herein.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE) | **Last Amended**: 2025-12-17
