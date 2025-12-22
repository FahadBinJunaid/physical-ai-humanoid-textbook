from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from config import settings


# Create database engine
engine = create_engine(
    settings.database_url,
    pool_pre_ping=True,  # Verify connections are alive before using them
    pool_recycle=300,    # Recycle connections after 5 minutes
    echo=False           # Set to True to see SQL queries for debugging
)


# Create session factory
SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine
)


# Dependency to get database session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Create all tables in the database
def create_tables():
    from models import Base
    Base.metadata.create_all(bind=engine)