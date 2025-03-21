from ultralytics import YOLO

MODEL_PATH = "models/best_full_integer_quant_edgetpu.tflite"

class ObjectDetector:
    """Computer vision system using YOLO for object detection"""
    
    def __init__(self, model_path=MODEL_PATH):
        """Initialize the vision system with specified model"""
        self.detections = None
        self.model = YOLO(model_path)
        
    def process_frame(self, image_frame):
        """Process an image frame and update detection results"""
        if image_frame is not None:
            self.detections = self.model.predict(image_frame)
        
    def get_results(self):
        """Retrieve the latest detection results"""
        return self.detections
