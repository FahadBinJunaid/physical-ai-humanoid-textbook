from typing import List, Dict, Any
from langchain_core.tools import tool
from retriever import Retriever
from schemas import SourceDocument


class ToolManager:
    """
    Class to manage tools available to the agent
    """

    def __init__(self):
        self.retriever = Retriever()

    def get_tools(self) -> List[Dict[str, Any]]:
        """
        Get all available tools as a list of dictionaries
        """
        return [
            {
                "name": "retrieve",
                "description": "Retrieve relevant documents from the knowledge base based on the query",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query to find relevant documents"
                        },
                        "top_k": {
                            "type": "integer",
                            "description": "Number of documents to retrieve (default: 5)"
                        }
                    },
                    "required": ["query"]
                }
            }
        ]

    def retrieve_tool(self, query: str, top_k: int = 5) -> List[Dict[str, str]]:
        """
        Tool function to retrieve relevant documents
        """
        source_docs = self.retriever.retrieve_relevant_documents(query, top_k)

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


# Individual tool functions that can be used directly
@tool
def retrieve(query: str, top_k: int = 5) -> List[Dict[str, str]]:
    """
    Retrieve relevant documents from the knowledge base based on the query
    """
    tool_manager = ToolManager()
    return tool_manager.retrieve_tool(query, top_k)