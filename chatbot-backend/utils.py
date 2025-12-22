import uuid
from typing import Optional
from pydantic import UUID4


def generate_uuid() -> UUID4:
    """
    Generate a new UUID4
    """
    return uuid.uuid4()


def validate_uuid(uuid_string: str) -> Optional[UUID4]:
    """
    Validate if a string is a valid UUID4
    """
    try:
        return uuid.UUID(uuid_string)
    except ValueError:
        return None


def generate_session_token() -> str:
    """
    Generate a unique session token
    """
    return str(uuid.uuid4())


def is_valid_session_token(token: str) -> bool:
    """
    Validate if a session token is properly formatted
    """
    try:
        uuid.UUID(token)
        return True
    except ValueError:
        return False