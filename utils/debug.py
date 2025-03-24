import logging
import os
import time
from enum import Enum

class DebugLevel(Enum):
    NONE = 0
    ERROR = 1
    WARNING = 2
    INFO = 3
    DEBUG = 4
    VERBOSE = 5

class QuaxiDebug:
    """Debug and logging system for Quaxi autonomous vehicle"""
    
    def __init__(self, level=DebugLevel.INFO, log_to_file=True):
        """Initialize the debug system"""
        self.level = level
        self.log_to_file = log_to_file
        self.performance_metrics = {}
        self.start_times = {}
        
        # Setup logging
        log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        self.logger = logging.getLogger('Quaxi')
        
        # Configure logging level
        numeric_level = getattr(logging, self._convert_level(level).name)
        self.logger.setLevel(numeric_level)
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(logging.Formatter(log_format))
        self.logger.addHandler(console_handler)
        
        # File handler if enabled
        if log_to_file:
            log_dir = "/Users/olivermorrow/Quaxi/logs"
            os.makedirs(log_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            log_file = f"{log_dir}/quaxi_{timestamp}.log"
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(logging.Formatter(log_format))
            self.logger.addHandler(file_handler)
    
    def _convert_level(self, debug_level):
        """Convert our debug levels to logging levels"""
        mapping = {
            DebugLevel.NONE: logging.NOTSET,
            DebugLevel.ERROR: logging.ERROR,
            DebugLevel.WARNING: logging.WARNING,
            DebugLevel.INFO: logging.INFO,
            DebugLevel.DEBUG: logging.DEBUG,
            DebugLevel.VERBOSE: logging.DEBUG
        }
        return mapping.get(debug_level, logging.INFO)
    
    def log(self, message, level=DebugLevel.INFO):
        """Log a message with specified level"""
        if self.level.value >= level.value:
            log_method = getattr(self.logger, level.name.lower())
            log_method(message)
    
    def start_timer(self, operation_name):
        """Start timing an operation for performance tracking"""
        self.start_times[operation_name] = time.time()
    
    def end_timer(self, operation_name):
        """End timing for an operation and record metric"""
        if operation_name in self.start_times:
            elapsed = time.time() - self.start_times[operation_name]
            self.performance_metrics[operation_name] = elapsed
            if self.level.value >= DebugLevel.DEBUG.value:
                self.log(f"Performance: {operation_name} took {elapsed:.4f} seconds", DebugLevel.DEBUG)
            return elapsed
        return None
    
    def get_stats(self):
        """Get current performance statistics"""
        return self.performance_metrics
    
    def error(self, message):
        """Log an error message"""
        self.log(message, DebugLevel.ERROR)
    
    def warning(self, message):
        """Log a warning message"""
        self.log(message, DebugLevel.WARNING)
    
    def info(self, message):
        """Log an info message"""
        self.log(message, DebugLevel.INFO)
    
    def debug(self, message):
        """Log a debug message"""
        self.log(message, DebugLevel.DEBUG)
    
    def verbose(self, message):
        """Log a verbose debug message"""
        self.log(message, DebugLevel.VERBOSE)
