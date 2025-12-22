from typing import List, Dict, Any, Optional
from llm import LLMClient
from retriever import Retriever
from tools import ToolManager, retrieve
from validation import ResponseValidator
from schemas import SourceDocument
from conversation_state import ConversationStateManager, get_conversation_state_manager
import json


class RAGAgent:
    """
    RAG (Retrieval-Augmented Generation) Agent that orchestrates retrieval and generation
    """

    def __init__(self):
        self.llm_client = LLMClient()
        self.retriever = Retriever()
        self.tool_manager = ToolManager()
        self.validator = ResponseValidator()
        self.conversation_manager = get_conversation_state_manager()

    def process_query(self, query: str, session_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a user query by retrieving relevant context and generating a response
        """
        # Retrieve relevant documents
        relevant_docs = self.retriever.retrieve_relevant_documents(query)

        # Generate response with context
        result = self.llm_client.generate_response_with_sources(query, relevant_docs)

        # Check if the response indicates a rate limit error
        if "rate limit" in result.get("response", "").lower() or "quota" in result.get("response", "").lower():
            # Return a user-friendly error message
            return {
                "response": "The chatbot is currently experiencing high demand. Please wait a moment and try again.",
                "sources": []
            }

        return result

    def process_query_with_tool_calling(self, query: str) -> Dict[str, Any]:
        """
        Process a query using tool-calling approach (simulated for now)
        In a full implementation, this would use the LLM's tool calling capabilities
        """
        # For now, we'll simulate tool calling by retrieving documents first
        retrieved_docs = self.retriever.retrieve_relevant_documents(query)

        if retrieved_docs:
            # Generate response based on retrieved documents
            result = self.llm_client.generate_response_with_sources(query, retrieved_docs)

            # Check if the response indicates a rate limit error
            if "rate limit" in result.get("response", "").lower() or "quota" in result.get("response", "").lower():
                # Return a user-friendly error message
                return {
                    "response": "The chatbot is currently experiencing high demand. Please wait a moment and try again.",
                    "sources": []
                }
        else:
            # If no documents found, generate a response indicating this
            result = {
                "response": "I couldn't find relevant information in the documentation to answer your question. Please try rephrasing your question or check if the topic is covered in the robotics handbook.",
                "sources": []
            }

        return result

    def process_query_with_citation(self, query: str) -> Dict[str, Any]:
        """
        Process a query and ensure proper source citation in responses
        """
        try:
            # Check if the query is a greeting to avoid searching for irrelevant context
            greeting_keywords = ["hello", "hi", "hey", "greetings", "good morning", "good afternoon", "good evening", "morning", "afternoon", "evening"]
            query_lower = query.lower().strip()
            is_greeting = any(greeting in query_lower for greeting in greeting_keywords)

            if is_greeting:
                # For greetings, don't retrieve documents, just generate a friendly response
                response_text = self.llm_client.generate_response(query, None)

                # Check if the response indicates a rate limit error
                if "rate limit" in response_text.lower() or "quota" in response_text.lower():
                    # Return a user-friendly error message
                    return {
                        "response": "Hello! I'm currently experiencing high demand. Please wait a moment and try again.",
                        "sources": []
                    }
                else:
                    # Early return for greetings to avoid RAG pipeline, citations, and validation notes
                    return {
                        "response": response_text,
                        "sources": []
                    }

            # For non-greetings, continue with the strict RAG pipeline
            # Always query Qdrant for relevant documents, regardless of topic
            retrieved_docs = self.retriever.retrieve_relevant_documents(query)

            if retrieved_docs:
                # Generate response with explicit source citations using retrieved context
                result = self.llm_client.generate_response_with_sources(query, retrieved_docs)

                # Check if the response indicates a rate limit error
                if "rate limit" in result.get("response", "").lower() or "quota" in result.get("response", "").lower():
                    # Return a user-friendly error message
                    return {
                        "response": "The chatbot is currently experiencing high demand. Please wait a moment and try again.",
                        "sources": []
                    }

                # Validate that the response is properly grounded in the sources
                sources = result.get("sources", [])
                response_text = result.get("response", "")

                validation_result = self.validator.validate_response_grounding(response_text, sources)
                # Return the result with sources for frontend display
            else:
                # No relevant documents found, return response with empty sources
                result = {
                    "response": "I am sorry, but I can only answer questions based on the provided Robotics Handbook. The system did not find relevant information in the documentation to answer your question. Please try rephrasing your question or check if the topic is covered in the robotics handbook.",
                    "sources": []
                }
        except Exception as e:
            # Handle any errors during the retrieval or generation process
            error_msg = str(e).lower()
            if "rate limit" in error_msg or "quota" in error_msg or "429" in error_msg:
                return {
                    "response": "The chatbot is currently experiencing high demand. Please wait a moment and try again.",
                    "sources": []
                }
            else:
                return {
                    "response": f"An error occurred while processing your request: {str(e)}. Please try again or rephrasing your question.",
                    "sources": []
                }

        return result

    def _is_follow_up_query(self, query: str, conversation_history: List[Dict[str, str]]) -> bool:
        """
        Determine if the current query is a follow-up to the conversation history
        """
        if not conversation_history:
            return False

        # Check if query contains follow-up indicators
        follow_up_indicators = [
            'it', 'that', 'this', 'he', 'she', 'they', 'them', 'him', 'her',
            'previous', 'above', 'earlier', 'mentioned', 'said', 'asked',
            'you mentioned', 'as you said', 'according to you',
            'the same', 'similar', 'like', 'also', 'too'
        ]

        query_lower = query.lower()

        # Check for pronouns and follow-up terms that suggest reference to previous context
        for indicator in follow_up_indicators:
            if indicator in query_lower:
                return True

        # Check if query seems to reference something from the conversation history
        # by looking for potential entity matches
        for exchange in conversation_history:
            content = exchange.get('content', '').lower()
            # Simple heuristic: if query contains terms that appeared in history
            query_words = set(query_lower.split())
            history_words = set(content.split())
            common_words = query_words.intersection(history_words)

            # If there are common terms and query is relatively short (likely a follow-up)
            if len(common_words) > 1 and len(query.split()) < 10:
                return True

        return False

    def chat_with_context(self, query: str, conversation_history: List[Dict[str, str]] = None) -> Dict[str, Any]:
        """
        Process a query while considering conversation context
        """
        if not conversation_history:
            # If no history, just process the current query
            return self.process_query_with_citation(query)

        # Create a context string from recent conversation history
        # Limit to last few exchanges to avoid exceeding token limits
        recent_history = conversation_history[-3:]  # Use last 3 exchanges
        context_str = "Recent conversation history:\n"
        for exchange in recent_history:
            role = exchange.get('role', 'user')
            content = exchange.get('content', '')
            timestamp = exchange.get('timestamp', '')
            context_str += f"{role.capitalize()}: {content}\n"

        # Identify if this is a follow-up question based on context
        is_follow_up = self._is_follow_up_query(query, recent_history)

        if is_follow_up:
            # Enhance the query with conversation context for better understanding
            enhanced_prompt = f"""{context_str}

Current question: {query}

Based on the conversation history, please provide a response that acknowledges the context and maintains continuity with the previous exchanges. If the current question refers to information from the history, please incorporate that information into your response."""
        else:
            # Standard contextualized query
            enhanced_prompt = f"""{context_str}

Current question: {query}

Please provide an accurate answer based on the robotics documentation. If the conversation history is relevant to answering this question, please consider it in your response."""

        # Process the enhanced query with context
        return self.process_query_with_citation(enhanced_prompt)

    def get_conversation_context_window(self, conversation_history: List[Dict[str, str]], max_context_length: int = 2000) -> List[Dict[str, str]]:
        """
        Get a window of conversation history that fits within the context length limit
        """
        if not conversation_history:
            return []

        # Start from the most recent and work backwards until we reach the length limit
        context_window = []
        current_length = 0

        for exchange in reversed(conversation_history):
            exchange_str = f"{exchange.get('role', 'user')}: {exchange.get('content', '')}"
            if current_length + len(exchange_str) > max_context_length:
                break

            context_window.insert(0, exchange)  # Insert at beginning to maintain order
            current_length += len(exchange_str)

        return context_window

    def summarize_conversation_context(self, conversation_history: List[Dict[str, str]], max_turns: int = 10) -> List[Dict[str, str]]:
        """
        Summarize long conversations to maintain performance by limiting the number of turns
        """
        if not conversation_history or len(conversation_history) <= max_turns:
            return conversation_history

        # For now, we'll take the first and last few exchanges to preserve context
        # In a more sophisticated implementation, we would use an LLM to generate summaries
        if len(conversation_history) > max_turns:
            # Keep first 2 exchanges and last (max_turns-2) exchanges
            first_exchanges = conversation_history[:2]
            last_exchanges = conversation_history[-(max_turns-2):]
            return first_exchanges + last_exchanges

        return conversation_history

    def generate_context_summary(self, conversation_history: List[Dict[str, str]]) -> str:
        """
        Generate a textual summary of the conversation context
        """
        if not conversation_history:
            return ""

        # For now, create a simple concatenation of key exchanges
        # In a real implementation, this would use an LLM to generate a proper summary
        summary_parts = ["Conversation Summary:"]

        # Include first exchange if it exists
        if conversation_history:
            first_exchange = conversation_history[0]
            summary_parts.append(f"Started with: {first_exchange.get('content', '')[:100]}...")

        # Include last few exchanges
        recent_exchanges = conversation_history[-3:]  # Last 3 exchanges
        for exchange in recent_exchanges:
            role = exchange.get('role', 'user')
            content = exchange.get('content', '')
            summary_parts.append(f"{role}: {content[:50]}...")

        return "\n".join(summary_parts)

    def create_conversation_session(self, session_id: Optional[str] = None) -> str:
        """
        Create a new conversation session with state tracking
        """
        conversation = self.conversation_manager.create_conversation(session_id)
        return conversation.session_id

    def track_conversation_state(self, session_id: str, user_message: str, assistant_response: str) -> bool:
        """
        Track the state of a conversation by recording user and assistant messages
        """
        # Add user message to conversation state
        user_msg = self.conversation_manager.add_message_to_conversation(
            session_id, "user", user_message
        )

        # Add assistant response to conversation state
        assistant_msg = self.conversation_manager.add_message_to_conversation(
            session_id, "assistant", assistant_response
        )

        return user_msg is not None and assistant_msg is not None

    def get_conversation_context(self, session_id: str, max_messages: int = 5) -> List[Dict[str, str]]:
        """
        Get the conversation context for a given session
        """
        return self.conversation_manager.get_conversation_history(session_id, max_messages) or []

    def validate_response_grounding(self, response: str, sources: List[SourceDocument]) -> bool:
        """
        Validate that the response is grounded in the provided sources
        This is a simplified check - in practice, this would involve more sophisticated validation
        """
        if not sources:
            # If no sources provided but response claims to be based on documentation, it's not grounded
            return "based on documentation" not in response.lower()

        # Simple check: ensure response doesn't contradict source availability
        has_sources = len(sources) > 0
        response_mentions_sources = any([
            "according to the documentation" in response.lower(),
            "the documentation states" in response.lower(),
            "based on the provided information" in response.lower()
        ])

        return True  # Simplified validation - in practice, implement more thorough checks