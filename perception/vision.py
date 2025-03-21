from ultralytics import YOLO
from vilib import Vilib

MODEL_PATH = "models/best_full_integer_quant_edgetpu.tflite"

class ObjectDetector:
    """Computer vision system using YOLO for object detection"""
    
    def __init__(self, model_path=MODEL_PATH):
        """Initialize the vision system with specified model"""
        self.detections = None
        self.model = YOLO(model_path)
        # Enable built-in object detection from vilib if needed
        # Vilib.object_detect_switch(True)
        
    def process_frame(self, image_frame):
        """Process an image frame and update detection results"""
        if image_frame is not None:
            self.detections = self.model.predict(image_frame)
        
    def get_results(self):
        """Retrieve the latest detection results"""
        return self.detections
        
    def get_vilib_detections(self):
        """Get detections from vilib's built-in detection"""
        if Vilib.detect_obj_parameter is not None:
            return Vilib.detect_obj_parameter
        return None