# Tasks Checklist

## Phase 1: Setup & Content Structure
- [ ] **Init:** Clean up default Docusaurus boilerplate (remove tutorial files).
- [ ] **Config:** Update `docusaurus.config.ts` with project metadata (Panaversity, Title, URL).
- [ ] **Structure:** Create folders in `/docs`:
    - `/docs/01-intro`
    - `/docs/02-module-1-ros2`
    - `/docs/03-module-2-simulation`
    - `/docs/04-module-3-isaac`
    - `/docs/05-module-4-vla`
- [ ] **Content:** Write `intro.md` explaining the Hardware Requirements (RTX, Jetson).
- [ ] **Content:** Write a detailed `module-1/ros2-nodes.md` (as a sample for testing).

## Phase 2: UI/UX Redesign
- [ ] **Styling:** Install Tailwind CSS into Docusaurus.
- [ ] **Landing Page:** Redesign `src/pages/index.tsx` with a Hero section and Course Curriculum grid.
- [ ] **Theme:** Change primary colors in `src/css/custom.css` to "Cyber Robotics" theme (Dark/Neon).

## Phase 3: RAG Implementation
- [ ] **Backend:** Create a `/backend` folder (Python/FastAPI).
- [ ] **Ingest:** Write `ingest.py` to read `/docs` and upload to Qdrant.
- [ ] **Frontend:** Build a `ChatBot.tsx` component using Shadcn UI or custom CSS.
- [ ] **Integration:** Add the ChatBot component to the global `Layout` wrapper.

## Phase 4: Bonus Features (Auth & Actions)
- [ ] **Auth:** Setup Better-Auth and create `SignupForm.tsx`.
- [ ] **Profile:** Create a User Context to store "Beginner/Advanced" status.
- [ ] **Feature:** Create `ActionButtons.tsx` (Personalize / Translate) that injects into Markdown pages.
- [ ] **Logic:** Implement the API call for Personalization (sending current page text -> LLM -> returning new text).

## Phase 5: Deployment
- [ ] **Build:** Run `npm run build` locally to check for broken links.
- [ ] **Docs:** Ensure `README.md` explains how to run the backend and frontend.