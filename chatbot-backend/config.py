from pydantic_settings import BaseSettings
from typing import Optional
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere
import google.generativeai as genai

# Load environment variables from .env file
load_dotenv()

# Manually load specific environment variables
COHERE_API_KEY = os.environ.get("COHERE_API_KEY")
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY")
QDRANT_URL = os.environ.get("QDRANT_URL")
QDRANT_API_KEY = os.environ.get("QDRANT_API_KEY")


class Settings(BaseSettings):
    # Qdrant Configuration
    qdrant_url: str = QDRANT_URL or "https://41561210-ab70-4c14-80a1-7d0a1aae50f8.europe-west3-0.gcp.cloud.qdrant.io"
    qdrant_api_key: Optional[str] = QDRANT_API_KEY

    # Cohere Configuration
    cohere_api_key: Optional[str] = COHERE_API_KEY

    # Google Gemini Configuration
    gemini_api_key: Optional[str] = GEMINI_API_KEY

    # OpenRouter Configuration
    openrouter_api_key: Optional[str] = os.environ.get("OPENROUTER_API_KEY")
    openrouter_base_url: str = os.environ.get("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")

    # Database Configuration
    database_url: str = os.environ.get("DATABASE_URL", "postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/your_db_name?sslmode=require")

    # Application Configuration
    chat_model_name: str = os.environ.get("CHAT_MODEL_NAME", "gemini-2.0-flash")
    embedding_model_name: str = "embed-english-v3.0"
    collection_name: str = "robotics_docs"
    chunk_size: int = 1000
    chunk_overlap: int = 100

    # Security
    secret_key: str = "your_secret_key_here"
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # Server Configuration
    host: str = "0.0.0.0"
    port: int = 8000

    model_config = {
        "env_file": ".env",
        "case_sensitive": True,
        "extra": "ignore"
    }


# Create settings instance
settings = Settings()


# Initialize Qdrant client
def get_qdrant_client():
    return QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        prefer_grpc=False  # Using REST API
    )


# Initialize Cohere client
def get_cohere_client():
    if not settings.cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable is not set")
    return cohere.Client(api_key=settings.cohere_api_key)


# Initialize Gemini client
def configure_gemini():
    if not settings.gemini_api_key:
        raise ValueError("GEMINI_API_KEY environment variable is not set")
    print("DEBUG: Using API Key starting with:", os.getenv('GEMINI_API_KEY')[:5])
    genai.configure(api_key=settings.gemini_api_key)


# Get configured Gemini model
def get_gemini_model():
    configure_gemini()
    return genai.GenerativeModel(settings.chat_model_name)