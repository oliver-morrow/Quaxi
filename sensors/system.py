import cv2
import numpy as np
import threading
from picamera2 import Picamera2
from perception.vision import ObjectDetector
from sensors.line_detector import get_line_position
from sensors.distance_sensor import measure_distance

def vision_processing_thread(sensor_system):
    """Background thread for continuous visual processing"""
    while sensor_system.is_running:
        # Capture frame and process with AI
        frame = sensor_system.camera.capture_array()
        sensor_system.vision_module.process_frame(frame)

class IntegratedSensorSystem:
    """Manages all sensor subsystems on the vehicle"""
    
    def __init__(self, vehicle):
        """Initialize with a reference to the vehicle"""
        self.vehicle = vehicle
        self.is_running = False
        self.vision_module = ObjectDetector()
        
        # Initialize camera
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_still_configuration(
            main={"size": (640, 640), "format": "RGB888"}))
        self.camera.start()
        
        # Thread for vision processing
        self.vision_thread = None
    
    def get_vision_detections(self):
        """Retrieve latest object detection results"""
        return self.vision_module.detections
    
    def get_environmental_data(self):
        """Collect data from all environmental sensors"""
        sensor_readings = {
            "obstacle_distance": measure_distance(self.vehicle),
            "lane_position": get_line_position(self.vehicle)
        }
        return sensor_readings
    
    def analyze_scene(self):
        """Perform general scene analysis on camera input"""
        # Get current frame
        current_image = self.camera.capture_array()
        if current_image is None:
            return 0
        
        # Basic image analysis pipeline
        grayscale = cv2.cvtColor(current_image, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(grayscale, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Compute scene complexity metric
        edge_density = np.count_nonzero(edges)
        complexity_score = min(100, edge_density / 500)
        
        return complexity_score
    
    def begin_monitoring(self):
        """Start all sensor monitoring systems"""
        self.is_running = True
        self.vision_thread = threading.Thread(target=vision_processing_thread, args=(self,))
        self.vision_thread.daemon = True
        self.vision_thread.start()
    
    def end_monitoring(self):
        """Properly shut down all sensor systems"""
        self.is_running = False
        if hasattr(self, 'vision_thread') and self.vision_thread.is_alive():
            self.vision_thread.join(timeout=1.0)
