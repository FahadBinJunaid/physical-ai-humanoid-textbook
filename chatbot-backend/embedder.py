from typing import List
from config import get_cohere_client, settings


class Embedder:
    """
    Class to handle text embedding using Cohere
    """

    def __init__(self):
        self.client = get_cohere_client()
        self.model_name = settings.embedding_model_name

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        if not texts:
            return []

        response = self.client.embed(
            texts=texts,
            model=self.model_name,
            input_type="search_document"
        )

        return response.embeddings

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a single query
        """
        if not query:
            return []

        response = self.client.embed(
            texts=[query],
            model=self.model_name,
            input_type="search_query"
        )

        return response.embeddings[0] if response.embeddings else []