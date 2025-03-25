from aiymakerkit import vision
from pycoral.utils.dataset import read_label_file
import os.path
import numpy as np

def get_model_path(filename):
    """Returns the full path to a model file in the models directory."""
    root_dir = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(root_dir, 'models', filename)

class AISensor:
    """
    AI-based object detection sensor using AIY Maker Kit with Edge TPU.
    Provides object detection capabilities with a simple, clean interface.
    """
    def __init__(self):
        # Model and label file paths
        self.model_path = get_model_path('pascal_label.tflite')
        self.label_path = get_model_path('pascal_label.txt')
        
        # Initialize detector with error handling
        try:
            self.detector = vision.Detector(self.model_path)
            self.labels = read_label_file(self.label_path)
            self.is_detector_available = True
        except Exception as e:
            print(f"Error initializing AI detector: {e}")
            # Create dummy detector and labels for graceful degradation
            self.detector = None
            self.labels = {}
            self.is_detector_available = False
            
        # Detection state
        self.results = None
        self.ready = False
        
        # Object class names
        self.classes = ['duck_regular', 'sign_noentry', 'sign_oneway_left', 
                       'sign_oneway_right', 'sign_stop', 'sign_yield']

    def run(self, frame=None):
        """
        Process a frame to detect objects.
        If no frame is provided, try to get one from AIY vision system.
        """
        try:
            # Skip processing if detector isn't available
            if not self.is_detector_available:
                self.results = None
                return
                
            # Get frame from camera if none was provided
            if frame is None:
                frames = list(vision.get_frames(max_frames=1))
                if frames:
                    frame = frames[0]
                else:
                    return
            
            # Detect objects in the frame
            detected_objects = self.detector.get_objects(frame, threshold=0.4)
            
            # Format the results in a clean structure
            if detected_objects:
                self.results = self._process_detections(frame, detected_objects)
            else:
                self.results = None
                
        except Exception as e:
            print(f"Error in AI processing: {e}")
            self.results = None
        finally:
            self.ready = True

    def _process_detections(self, frame, objects):
        """
        Process the raw detection results into a simple format.
        Returns a list of detected objects with their properties.
        """
        height, width = frame.shape[:2]
        processed_detections = []
        
        for obj in objects:
            # Get bounding box coordinates
            x1, y1, x2, y2 = obj.bbox
            
            # Map the label to our class system
            label_id = obj.id
            class_index = self._map_label_to_class_index(label_id)
            
            # Create a simple detection object
            detection = {
                'bbox': (x1, y1, x2, y2),
                'center': ((x1 + x2) / 2, (y1 + y2) / 2),
                'size': (x2 - x1, y2 - y1),
                'normalized_center': ((x1 + x2) / 2 / width, (y1 + y2) / 2 / height),
                'class': class_index,
                'label': label_id,
                'confidence': obj.score
            }
            
            processed_detections.append(detection)
            
        return processed_detections

    def _map_label_to_class_index(self, label_id):
        """Map AIY Maker Kit label to our class indices"""
        class_mapping = {
            "stop": 4,           # sign_stop
            "yield": 5,          # sign_yield
            "duck": 0,           # duck_regular
            "no_entry": 1,       # sign_noentry
            "oneway_left": 2,    # sign_oneway_left
            "oneway_right": 3    # sign_oneway_right
        }
        
        # Default to duck (0) if label not found in mapping
        return class_mapping.get(label_id, 0)

    def read(self):
        """Return the latest detection results"""
        return self.results
