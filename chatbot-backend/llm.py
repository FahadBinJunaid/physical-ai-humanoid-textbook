from typing import List, Dict, Any, Optional
from config import settings
from schemas import SourceDocument
from langchain_openai import ChatOpenAI
from langchain_core.tools import tool
import time
import random


class LLMClient:
    """
    Class to handle interaction with OpenRouter LLM
    """

    def __init__(self):
        self.model = ChatOpenAI(
            base_url=settings.openrouter_base_url,
            api_key=settings.openrouter_api_key,
            model=settings.chat_model_name,
            temperature=0.0,  # Zero temperature to prevent creativity/hallucinations
            default_headers={
                "HTTP-Referer": "http://localhost:7860",  # Updated for Hugging Face compatibility
                "X-Title": "Robotics Chatbot"
            }
        )

    def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """
        Generate a response using the LLM with strict RAG implementation
        """
        # Check if the prompt is a greeting
        greeting_keywords = ["hello", "hi", "hey", "greetings", "good morning", "good afternoon", "good evening", "morning", "afternoon", "evening"]
        prompt_lower = prompt.lower().strip()

        # Check if it's a greeting (simple check)
        is_greeting = any(greeting in prompt_lower for greeting in greeting_keywords)

        if is_greeting:
            # Handle greetings with a friendly response without context
            full_prompt = f"""You are a strict Robotics Assistant. You have NO knowledge outside the provided context retrieved from the database. If a user asks about general history, math, or any topic NOT present in the retrieved context, you must politely state: "I am sorry, but I can only answer questions based on the provided Robotics Handbook." Do NOT guess or use your internal training data.

Someone just greeted you with: "{prompt}"

Please respond with a warm, friendly greeting and offer help with robotics topics."""
        else:
            # For non-greetings, use strict context policy
            if context:
                full_prompt = f'''You are a strict Robotics Assistant. You have NO knowledge outside the provided context retrieved from the database. If a user asks about general history, math, or any topic NOT present in the retrieved context, you must politely state: "I am sorry, but I can only answer questions based on the provided Robotics Handbook." Do NOT guess or use your internal training data.

Here is the relevant context from the robotics documentation:
{context}

Question: {prompt}

ANSWER using ONLY the provided context. Reference the source documents directly. Do NOT make up information. Do NOT add any facts not in the context. Be helpful but stick strictly to the provided information. Always mention which document the information comes from. If you don't know about something because it's not in the context, explicitly state: "I am sorry, but I can only answer questions based on the provided Robotics Handbook."'''
            else:
                # No context available - respond with the strict policy
                full_prompt = f'''You are a strict Robotics Assistant. You have NO knowledge outside the provided context retrieved from the database. If a user asks about general history, math, or any topic NOT present in the retrieved context, you must politely state: "I am sorry, but I can only answer questions based on the provided Robotics Handbook." Do NOT guess or use your internal training data.

Question: {prompt}

I am sorry, but I can only answer questions based on the provided Robotics Handbook. The system did not find relevant information in the documentation to answer your question. Please try rephrasing your question or check if the topic is covered in the robotics handbook.'''

        # Exponential backoff retry mechanism for rate limit errors
        max_retries = 3
        base_delay = 1  # Base delay in seconds
        max_delay = 60  # Maximum delay in seconds

        for attempt in range(max_retries + 1):
            try:
                from langchain_core.messages import HumanMessage
                message = HumanMessage(content=full_prompt)
                response = self.model.invoke([message])
                return response.content
            except Exception as e:
                error_msg = str(e)

                # Check if it's a rate limit or quota exceeded error
                if "429" in error_msg or "quota" in error_msg.lower() or "rate limit" in error_msg.lower() or "exceeded" in error_msg.lower():
                    if attempt < max_retries:
                        # Calculate exponential backoff with jitter
                        delay = min(base_delay * (2 ** attempt), max_delay)
                        jitter = random.uniform(0, delay * 0.1)  # Add 10% jitter
                        actual_delay = delay + jitter

                        print(f"Rate limit hit, retrying in {actual_delay:.2f} seconds... (attempt {attempt + 1}/{max_retries})")
                        time.sleep(actual_delay)
                        continue
                    else:
                        # All retries exhausted
                        return f"Rate limit exceeded. Please try again later. Error: {str(e)}"
                else:
                    # Not a rate limit error, return immediately
                    return f"Error generating response: {str(e)}"

    def generate_response_with_sources(self, prompt: str, sources: List[SourceDocument]) -> Dict[str, Any]:
        """
        Generate a response with source citations
        """
        # Check if the prompt is a greeting
        greeting_keywords = ["hello", "hi", "hey", "greetings", "good morning", "good afternoon", "good evening", "morning", "afternoon", "evening"]
        prompt_lower = prompt.lower().strip()

        # Check if it's a greeting (simple check)
        is_greeting = any(greeting in prompt_lower for greeting in greeting_keywords)

        if is_greeting:
            # For greetings, bypass the RAG process and return a friendly response with empty sources
            response_text = self.generate_response(prompt, None)
            return {
                "response": response_text,
                "sources": []  # Empty sources for greetings
            }

        # Create context from sources for non-greetings
        context_parts = []
        for source in sources:
            context_parts.append(f"Document: {source.title}\nContent: {source.content}\n")

        context = "\n\n".join(context_parts) if context_parts else ""

        # Generate response
        response_text = self.generate_response(prompt, context)

        return {
            "response": response_text,
            "sources": sources
        }

    async def generate_with_tool_calling(self, prompt: str, tools: List[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate response with potential tool calling capability
        """
        # Check if the prompt is a greeting
        greeting_keywords = ["hello", "hi", "hey", "greetings", "good morning", "good afternoon", "good evening", "morning", "afternoon", "evening"]
        prompt_lower = prompt.lower().strip()

        # Check if it's a greeting (simple check)
        is_greeting = any(greeting in prompt_lower for greeting in greeting_keywords)

        if is_greeting:
            # For greetings, return a friendly response with empty sources
            response = self.generate_response(prompt, None)
            return {
                "response": response,
                "sources": []  # Empty sources for greetings
            }

        # For non-greetings, use the existing approach
        response = self.generate_response(prompt)

        return {
            "response": response,
            "sources": []  # Will be populated when we integrate with retriever
        }


# Tool definition for retrieval
@tool
def retrieve(query: str, top_k: int = 3) -> List[Dict[str, str]]:
    """
    Retrieve relevant documents from the knowledge base based on the query
    """
    from retriever import Retriever
    retriever = Retriever()
    source_docs = retriever.retrieve_relevant_documents(query, top_k)

    # Convert SourceDocument to dict format for the tool
    return [
        {
            "document": doc.document,
            "title": doc.title,
            "content": doc.content,
            "score": str(doc.score)
        }
        for doc in source_docs
    ]