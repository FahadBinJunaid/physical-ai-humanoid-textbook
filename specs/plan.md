# Implementation Plan (The "How")

## Architecture
1.  **Client:** Docusaurus Static Site (React/TS).
    * `src/components`: Custom React components for Chat, Auth Forms, and Buttons.
    * `docs/`: Markdown files for the book.
2.  **Server (API):** Python FastAPI (hosted separately or serverless functions).
    * `/api/chat`: Endpoint for RAG.
    * `/api/personalize`: Endpoint for content rewriting.
    * `/api/translate`: Endpoint for Urdu translation.
3.  **Database:**
    * **Vector Store:** Qdrant (for indexing the book content).
    * **Relational:** Neon Postgres (for User tables).

## Development Roadmap

### Phase 1: Foundation & Content (The Skeleton)
* Setup Docusaurus with TypeScript.
* Install Tailwind CSS for custom styling.
* Create the file structure for all Modules (1-4).
* Populate dummy content + real technical content for the first few chapters (to test RAG).

### Phase 2: Design & UI Polish
* Customize the Navbar and Footer.
* Create a custom Landing Page (`src/pages/index.tsx`).
* Ensure the site looks professional and "Production Ready".

### Phase 3: The Intelligence (RAG Backend)
* Set up Qdrant.
* Write a script to "ingest" the Docusaurus markdown files into Qdrant.
* Build the Chat UI Component in React.
* Connect Chat UI to the RAG backend.

### Phase 4: Advanced Features (The Bonus Points)
* **Auth:** Integrate Better-Auth client-side.
* **Personalization:** Create the "Personalize" button and hook it to the LLM.
* **Translation:** Create the "Urdu" button.

### Phase 5: Deployment & Review
* Build test.
* Deploy to GitHub Pages/Vercel.
* Record Demo Video.