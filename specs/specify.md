# Specifications (The "What")

## 1. Content Specifications (The Book)
The Docusaurus instance must contain the following structured documentation (docs):

* **Landing Page:** A high-impact homepage explaining Physical AI with a "Start Learning" Call to Action (CTA).
* **Sidebar Structure:**
    * *Introduction:* Vision of Physical AI, Hardware Setup (RTX 4070+, Jetson, etc.).
    * *Module 1 (ROS 2):* 3-4 Chapters (Nodes, Python Agents, URDF).
    * *Module 2 (Simulation):* Gazebo basics, Unity integration.
    * *Module 3 (NVIDIA Isaac):* Isaac Sim, VSLAM, Nav2.
    * *Module 4 (VLA & LLMs):* Voice-to-Action, Cognitive Planning.
    * *Capstone:* The Autonomous Humanoid project guide.

## 2. Feature Specifications
### A. RAG Chatbot (Core)
* **UI:** A floating chat widget or a sidebar panel.
* **Functionality:**
    * Users can ask: "How do I install ROS 2?"
    * System retrieves relevant chunks from the book markdown files.
    * System answers using OpenAI API.
    * *Context Menu:* If a user highlights text, they can "Ask AI about this".

### B. Authentication & Profiling (Bonus)
* **Provider:** Better-Auth.
* **Onboarding:** Upon signup, a modal asks:
    * "What is your software background? (Python/C++/Beginner)"
    * "Do you have the hardware? (Sim-only/Jetson Kit/Full Robot)"
* **Storage:** Save user profile in Neon (Postgres).

### C. Personalization (Bonus)
* **Trigger:** A "Personalize this Chapter" button at the top of content pages.
* **Logic:**
    * If User = "Beginner", rewrite the intro to be simpler.
    * If User = "Sim-only", hide "Physical Wiring" sections.

### D. Localization (Bonus)
* **Trigger:** A "Translate to Urdu" button.
* **Logic:** Use an LLM or Subagent to translate the current visible markdown content into Urdu on the fly (or switch to pre-generated Urdu pages).

## 3. Design & UI Specifications
* **Theme:** Custom Docusaurus Swizzle.
* **Colors:** Dark mode default. Accents: Neon Green (NVIDIA vibes) and Cyber Blue.
* **Typography:** Modern sans-serif (Inter or Roboto) for readability; Monospace (JetBrains Mono) for code.