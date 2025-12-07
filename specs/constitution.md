# Context & Constitution

## Project Overview
**Project Name:** Physical AI & Humanoid Robotics Textbook
**Type:** AI-Native Textbook & Interactive Platform
**Hackathon:** Agent Hackathon (Panaversity)
**Goal:** Create a unified textbook project using Docusaurus that teaches "Physical AI & Humanoid Robotics". The site must feature an embedded RAG chatbot, user authentication, and content personalization.

## Core Pillars
1.  **Educational Content:** A complete course curriculum covering ROS 2, Gazebo/Unity, NVIDIA Isaac, and VLA (Vision-Language-Action).
2.  **AI Integration:** A RAG Chatbot (using OpenAI/ChatKit, FastAPI, Qdrant) that answers questions based on the book.
3.  **User Experience:** Signup/Signin (Better-Auth), Personalization based on user background, and Urdu translation.

## The Course Material (Source of Truth)
**Module 1: The Robotic Nervous System (ROS 2)**
* Middleware, Nodes, Topics, Services, URDF.
**Module 2: The Digital Twin (Gazebo & Unity)**
* Physics simulation, LiDAR/Depth Camera simulation.
**Module 3: The AI-Robot Brain (NVIDIA Isaac)**
* Isaac Sim, Isaac ROS (VSLAM), Nav2.
**Module 4: Vision-Language-Action (VLA)**
* OpenAI Whisper (Voice), LLM Cognitive Planning, Capstone Project.

## Tech Stack Constraints
* **Frontend/Content:** Docusaurus (React/TypeScript).
* **Deployment:** GitHub Pages / Vercel.
* **Backend (AI):** Python (FastAPI), Neon (Postgres), Qdrant (Vector DB).
* **Auth:** Better-Auth.
* **Development Methodology:** Spec-Driven Development (Spec-Kit Plus style).

## Design Philosophy
* **Clean & Futuristic:** The UI should reflect "High-Tech" and "Robotics".
* **Scannable:** Content must be easy to read with clear diagrams and code blocks.
* **Interactive:** The AI chat and personalization buttons should be seamlessly integrated, not intrusive.