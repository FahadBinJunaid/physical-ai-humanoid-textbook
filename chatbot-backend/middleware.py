from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import time
import logging


class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log incoming requests and their response times
    """
    def __init__(self, app):
        super().__init__(app)
        self.logger = logging.getLogger(__name__)

    async def dispatch(self, request: Request, call_next):
        start_time = time.time()

        # Log the incoming request
        self.logger.info(f"Request: {request.method} {request.url}")

        response: Response = await call_next(request)

        # Calculate response time
        response_time = time.time() - start_time

        # Log the response
        self.logger.info(f"Response: {response.status_code} in {response_time:.2f}s")

        return response