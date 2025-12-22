from typing import List, Dict, Any
from schemas import SourceDocument


class ResponseValidator:
    """
    Class to validate that responses are properly grounded in documentation
    """

    def __init__(self):
        pass

    def validate_response_grounding(self, response: str, sources: List[SourceDocument]) -> Dict[str, Any]:
        """
        Validate that the response is grounded in the provided sources
        """
        result = {
            "is_valid": True,
            "confidence_score": 1.0,
            "issues": [],
            "suggestions": []
        }

        # If no sources were used but the response claims to be based on documentation
        if not sources and self._mentions_documentation(response):
            result["is_valid"] = False
            result["confidence_score"] = 0.0
            result["issues"].append("Response claims to be based on documentation but no sources were provided")
            result["suggestions"].append("Either provide relevant sources or clarify that the response is general knowledge")

        # If sources were provided, check if response seems related
        elif sources:
            # Check if response contains content that appears to come from the sources
            source_content_exists = any(
                self._content_matches_source(response, source.content)
                for source in sources
            )

            if not source_content_exists:
                result["is_valid"] = False
                result["confidence_score"] = 0.3
                result["issues"].append("Response content doesn't appear to match the provided sources")
                result["suggestions"].append("Ensure the response directly references content from the provided sources")

        return result

    def _mentions_documentation(self, text: str) -> bool:
        """
        Check if text mentions being based on documentation
        """
        doc_indicators = [
            "according to the documentation",
            "the documentation states",
            "based on the provided information",
            "from the handbook",
            "as mentioned in",
            "document says",
            "source states"
        ]

        text_lower = text.lower()
        return any(indicator in text_lower for indicator in doc_indicators)

    def _content_matches_source(self, response: str, source_content: str) -> bool:
        """
        Check if response content appears to be related to source content
        This is a simplified check - in practice, you might use more sophisticated NLP techniques
        """
        response_lower = response.lower()
        source_lower = source_content.lower()

        # Check for common words between response and source
        response_words = set(response_lower.split())
        source_words = set(source_lower.split())

        # If at least 10% of the words in the response also appear in the source
        common_words = response_words.intersection(source_words)
        if len(common_words) > 0:
            overlap_ratio = len(common_words) / max(len(response_words), 1)
            if overlap_ratio >= 0.1:  # At least 10% overlap
                return True

        # Additional check: if key phrases from source appear in response
        source_sentences = source_lower.split('.')
        for sentence in source_sentences[:5]:  # Check first 5 sentences
            if len(sentence.strip()) > 10:  # Only check sentences with meaningful content
                if sentence.strip()[:50] in response_lower:  # Check first 50 chars of sentence
                    return True

        # Additional check: see if response contains key concepts from source
        # Look for important terms that might indicate the response is based on the source
        important_terms = ['node', 'topic', 'service', 'parameter', 'ros2', 'robot', 'communication', 'system']
        source_terms = [term for term in important_terms if term in source_lower]
        response_terms = [term for term in important_terms if term in response_lower]

        # If at least one important term appears in both source and response
        if source_terms and response_terms:
            matching_terms = set(source_terms).intersection(set(response_terms))
            if matching_terms:
                return True

        return False

    def validate_sources_relevance(self, query: str, sources: List[SourceDocument]) -> Dict[str, Any]:
        """
        Validate that the provided sources are relevant to the query
        """
        result = {
            "relevance_score": 0.0,
            "relevant_sources": [],
            "irrelevant_sources": []
        }

        if not sources:
            return result

        query_lower = query.lower()
        relevant_count = 0

        for source in sources:
            # Check if source content relates to query
            source_content_lower = (source.content + " " + source.title).lower()
            query_words = query_lower.split()

            matching_words = sum(1 for word in query_words if word in source_content_lower)
            relevance_score = matching_words / len(query_words) if query_words else 0

            if relevance_score > 0.1:  # If at least 10% of query words match
                result["relevant_sources"].append({
                    "source": source,
                    "relevance_score": relevance_score
                })
                relevant_count += 1
            else:
                result["irrelevant_sources"].append({
                    "source": source,
                    "relevance_score": relevance_score
                })

        result["relevance_score"] = relevant_count / len(sources) if sources else 0.0
        return result

    def validate_context_relevance(self, query: str, conversation_history: List[Dict[str, str]], response: str) -> Dict[str, Any]:
        """
        Validate that the response is relevant to both the current query and conversation context
        """
        result = {
            "is_contextually_relevant": True,
            "relevance_score": 1.0,
            "issues": [],
            "suggestions": []
        }

        if not conversation_history:
            # If no history, just check if response is relevant to query
            if not self._check_response_relevance_to_query(response, query):
                result["is_contextually_relevant"] = False
                result["relevance_score"] = 0.5
                result["issues"].append("Response doesn't seem relevant to the current query")
                result["suggestions"].append("Ensure the response addresses the user's query directly")
            return result

        # Check if response acknowledges context appropriately
        context_relevant = self._check_context_in_response(response, conversation_history, query)

        if not context_relevant:
            result["is_contextually_relevant"] = False
            result["relevance_score"] = 0.7
            result["issues"].append("Response doesn't properly acknowledge relevant conversation context")
            result["suggestions"].append("Consider the conversation history when formulating the response")

        return result

    def _check_response_relevance_to_query(self, response: str, query: str) -> bool:
        """
        Check if response is relevant to the current query
        """
        response_lower = response.lower()
        query_lower = query.lower()

        # Simple keyword matching approach
        query_words = set(query_lower.split())
        response_words = set(response_lower.split())

        # If at least 30% of query words appear in response (or vice versa)
        if query_words and response_words:
            common_words = query_words.intersection(response_words)
            if len(common_words) / len(query_words) >= 0.3:
                return True

        return False

    def _check_context_in_response(self, response: str, conversation_history: List[Dict[str, str]], current_query: str) -> bool:
        """
        Check if the response appropriately considers the conversation context
        """
        if not conversation_history:
            return True

        # Check if response addresses potential follow-up nature of the query
        response_lower = response.lower()
        current_query_lower = current_query.lower()

        # Look for contextual elements in the conversation history
        context_elements = self._extract_context_elements(conversation_history)

        # If the current query seems like a follow-up and context elements are relevant
        is_follow_up = self._is_follow_up_query(current_query_lower, context_elements)

        if is_follow_up:
            # Check if the response acknowledges previous context
            # This is a simplified check - in practice, you might use more sophisticated NLP
            return True  # For now, we'll consider it valid if it's a follow-up query

        return True

    def _extract_context_elements(self, conversation_history: List[Dict[str, str]]) -> List[str]:
        """
        Extract key elements from conversation history that might be relevant
        """
        elements = []
        for exchange in conversation_history[-5:]:  # Look at last 5 exchanges
            content = exchange.get('content', '').lower()
            # Extract potential entities or topics from the conversation
            elements.append(content[:100])  # Store first 100 chars as context element
        return elements

    def _is_follow_up_query(self, query: str, context_elements: List[str]) -> bool:
        """
        Determine if the query is likely a follow-up to previous conversation
        """
        follow_up_indicators = [
            'it', 'that', 'this', 'he', 'she', 'they', 'them', 'him', 'her',
            'previous', 'above', 'earlier', 'mentioned', 'said', 'asked'
        ]

        query_words = query.split()
        if not query_words:
            return False

        # Check if query starts with or contains follow-up indicators
        for indicator in follow_up_indicators:
            if indicator in query:
                return True

        return False