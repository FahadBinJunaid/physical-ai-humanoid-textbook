import time
import logging
import asyncio
from typing import Callable, Any
from functools import wraps


class PerformanceMonitor:
    """
    Class to monitor and track performance metrics
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.response_times = []
        self.target_response_time = 5.0  # 5 seconds target

    def measure_response_time(self, func: Callable) -> Callable:
        """
        Decorator to measure the response time of a function
        """
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                return result
            finally:
                response_time = time.time() - start_time
                self._record_response_time(response_time, func.__name__)

        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                response_time = time.time() - start_time
                self._record_response_time(response_time, func.__name__)

        # Return the appropriate wrapper based on function type
        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper

    def _record_response_time(self, response_time: float, function_name: str):
        """
        Record response time and log if it exceeds the target
        """
        self.response_times.append(response_time)

        if response_time > self.target_response_time:
            self.logger.warning(
                f"Performance alert: {function_name} took {response_time:.2f}s "
                f"which exceeds target of {self.target_response_time}s"
            )
        else:
            self.logger.info(f"{function_name} completed in {response_time:.2f}s")

    def get_average_response_time(self) -> float:
        """
        Get the average response time
        """
        if not self.response_times:
            return 0.0
        return sum(self.response_times) / len(self.response_times)

    def get_performance_stats(self) -> dict:
        """
        Get comprehensive performance statistics
        """
        if not self.response_times:
            return {
                "total_requests": 0,
                "average_response_time": 0.0,
                "max_response_time": 0.0,
                "min_response_time": 0.0,
                "requests_within_target": 0,
                "requests_exceeding_target": 0
            }

        within_target = sum(1 for rt in self.response_times if rt <= self.target_response_time)
        exceeding_target = len(self.response_times) - within_target

        return {
            "total_requests": len(self.response_times),
            "average_response_time": sum(self.response_times) / len(self.response_times),
            "max_response_time": max(self.response_times),
            "min_response_time": min(self.response_times),
            "requests_within_target": within_target,
            "requests_exceeding_target": exceeding_target,
            "performance_percentage": (within_target / len(self.response_times)) * 100 if self.response_times else 0
        }


# Global performance monitor instance
monitor = PerformanceMonitor()


def time_limited_execution(timeout: float = 5.0):
    """
    Decorator to enforce a timeout on function execution
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                response_time = time.time() - start_time
                if response_time > timeout:
                    logging.warning(f"Function {func.__name__} exceeded timeout of {timeout}s ({response_time:.2f}s)")
                return result
            except Exception as e:
                response_time = time.time() - start_time
                if response_time > timeout:
                    logging.warning(f"Function {func.__name__} exceeded timeout of {timeout}s ({response_time:.2f}s) before error")
                raise e

        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                response_time = time.time() - start_time
                if response_time > timeout:
                    logging.warning(f"Function {func.__name__} exceeded timeout of {timeout}s ({response_time:.2f}s)")
                return result
            except Exception as e:
                response_time = time.time() - start_time
                if response_time > timeout:
                    logging.warning(f"Function {func.__name__} exceeded timeout of {timeout}s ({response_time:.2f}s) before error")
                raise e

        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper

    return decorator