from ultralytics import YOLO
import cv2
import numpy as np
import io
import threading
import time
from vilib import Vilib

MODEL_PATH = "models/best_full_integer_quant_edgetpu.tflite"
OBJECT_CATEGORIES = [
    'duck_regular', 'sign_noentry', 'sign_oneway_left', 
    'sign_oneway_right', 'sign_stop', 'sign_yield'
]

class ObjectDetector:
    """Computer vision system using YOLO for object detection"""
    
    def __init__(self, model_path=MODEL_PATH, debug=None, visualizer=None):
        """Initialize the vision system with specified model"""
        self.detections = None
        self.processing = False
        self.last_processed = 0
        self.debug = debug
        self.visualizer = visualizer
        
        # Load model
        if self.debug:
            self.debug.start_timer("model_load")
        try:
            self.model = YOLO(model_path)
            if self.debug:
                self.debug.end_timer("model_load")
                self.debug.info(f"YOLO model loaded from {model_path}")
        except Exception as e:
            if self.debug:
                self.debug.error(f"Failed to load YOLO model: {e}")
            print(f"Error loading model: {e}")
            self.model = None
            
        # Initialize visualizer if provided
        if self.visualizer:
            self.visualizer.start(class_labels=OBJECT_CATEGORIES)
        
        # Setup vilib detection if needed
        # Vilib.object_detect_switch(True)
        
    def process_frame(self, image_frame):
        """Process an image frame and update detection results"""
        if image_frame is None or self.processing:
            return
            
        self.processing = True
        
        # Skip if we processed a frame too recently (limit processing rate)
        current_time = time.time()
        if current_time - self.last_processed < 0.1:  # Max 10 fps for detection
            self.processing = False
            return
            
        self.last_processed = current_time
        
        # Process the frame with YOLO
        if self.model:
            if self.debug:
                self.debug.start_timer("inference")
                
            try:
                self.detections = self.model.predict(image_frame)
                
                # Update visualizer if available
                if self.visualizer:
                    self.visualizer.update_detections(self.detections)
                    
                if self.debug:
                    inference_time = self.debug.end_timer("inference")
                    self.debug.debug(f"Inference completed in {inference_time:.4f}s with {len(self.detections)} results")
            except Exception as e:
                if self.debug:
                    self.debug.error(f"Inference error: {e}")
                    
        self.processing = False
        
    def get_results(self):
        """Retrieve the latest detection results"""
        return self.detections
        
    def get_vilib_detections(self):
        """Get detections from vilib's built-in detection"""
        if Vilib.detect_obj_parameter is not None:
            return Vilib.detect_obj_parameter
        return None
        
    def get_detected_objects(self):
        """Get a list of detected objects with class names and positions"""
        objects = []
        if self.detections is None or not hasattr(self.detections, 'boxes'):
            return objects
            
        for box in self.detections.boxes:
            # Skip low confidence
            if float(box.conf[0]) < 0.4:
                continue
                
            cls_id = int(box.cls[0])
            if cls_id < len(OBJECT_CATEGORIES):
                objects.append({
                    'class': OBJECT_CATEGORIES[cls_id],
                    'confidence': float(box.conf[0]),
                    'position': box.xywh[0].tolist()  # x, y, width, height
                })
                
        return objects
    
    def stop(self):
        """Clean up resources"""
        if self.visualizer:
            self.visualizer.stop()