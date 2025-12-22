from typing import List, Dict, Any
from config import get_qdrant_client, settings
from embedder import Embedder
from schemas import SourceDocument


class Retriever:
    """
    Class to handle retrieval of relevant documents from Qdrant vector database
    """

    def __init__(self):
        self.qdrant_client = get_qdrant_client()
        self.embedder = Embedder()
        self.collection_name = settings.collection_name

    def retrieve_relevant_documents(self, query: str, top_k: int = 3) -> List[SourceDocument]:
        """
        Retrieve the most relevant documents for a given query from the entire vector database
        This ensures strict RAG by fetching the most relevant chunks across all stored documents
        """
        # Generate embedding for the query
        query_embedding = self.embedder.embed_query(query)

        # Search in Qdrant across the entire collection to get the most relevant results
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            # This search operates across the entire collection by default
            # ensuring we get the most relevant chunks from all stored documents
        )

        # Convert results to SourceDocument format with proper metadata for frontend
        source_documents = []
        for result in search_results:
            payload = result.payload
            source_doc = SourceDocument(
                document=payload.get("source_document", ""),
                title=payload.get("document_title", ""),
                content=payload.get("chunk_text", ""),
                score=result.score  # Include the relevance score for transparency
            )
            source_documents.append(source_doc)

        return source_documents

    def retrieve_by_ids(self, ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve documents by their IDs
        """
        results = self.qdrant_client.retrieve(
            collection_name=self.collection_name,
            ids=ids,
            with_payload=True
        )

        documents = []
        for result in results:
            documents.append({
                "id": result.id,
                "payload": result.payload,
                "content": result.payload.get("chunk_text", "")
            })

        return documents