import os
from typing import List, Tuple
from pathlib import Path
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import UnstructuredMarkdownLoader
import markdown
from bs4 import BeautifulSoup
import uuid


class DocumentProcessor:
    """
    Class to process MDX files from the docs directory and split them into chunks
    """

    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 100):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            length_function=len,
            is_separator_regex=False,
        )

    def load_mdx_files(self, docs_path: str = "../docs") -> List[Tuple[str, str, str]]:
        """
        Load all MDX files from the specified directory and return (file_path, content, title)
        """
        mdx_files = []
        docs_dir = Path(docs_path)

        if not docs_dir.exists():
            print(f"Docs directory {docs_dir} does not exist")
            return mdx_files

        for mdx_file in docs_dir.rglob("*.mdx"):
            try:
                with open(mdx_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract title from the content (look for first H1 or filename)
                title = self._extract_title(content, mdx_file.name)

                # Convert MDX to plain text if needed, but for now just return the raw content
                # as it will be processed by the embedding model
                mdx_files.append((str(mdx_file), content, title))
            except Exception as e:
                print(f"Error reading {mdx_file}: {str(e)}")

        return mdx_files

    def _extract_title(self, content: str, filename: str) -> str:
        """
        Extract title from content or use filename as fallback
        """
        # Try to find the first H1 header in the content
        lines = content.split('\n')
        for line in lines[:10]:  # Check first 10 lines
            if line.strip().startswith('# '):
                title = line.strip()[2:]  # Remove '# ' prefix
                return title.strip()

        # If no H1 header found, use the filename without extension
        return Path(filename).stem.replace('_', ' ').replace('-', ' ').title()

    def split_document(self, content: str, source_path: str, title: str) -> List[dict]:
        """
        Split document content into chunks with metadata
        """
        # For MDX files, we'll use the raw content for now
        # The text splitter will handle the chunking
        chunks = self.text_splitter.split_text(content)

        chunk_data = []
        for idx, chunk in enumerate(chunks):
            chunk_data.append({
                'id': str(uuid.uuid4()),  # This will be used as the Qdrant point ID
                'chunk_text': chunk,
                'source_document': source_path,
                'document_title': title,
                'chunk_index': idx
            })

        return chunk_data

    def process_all_documents(self, docs_path: str = "../docs") -> List[dict]:
        """
        Process all MDX files in the docs directory and return all chunks
        """
        all_chunks = []
        mdx_files = self.load_mdx_files(docs_path)

        for file_path, content, title in mdx_files:
            try:
                chunks = self.split_document(content, file_path, title)
                all_chunks.extend(chunks)
                print(f"Processed {file_path}: {len(chunks)} chunks created")
            except Exception as e:
                print(f"Error processing {file_path}: {str(e)}")

        return all_chunks


# Example usage
if __name__ == "__main__":
    processor = DocumentProcessor(chunk_size=1000, chunk_overlap=100)
    chunks = processor.process_all_documents()
    print(f"Total chunks created: {len(chunks)}")