import os.path
import cv2
import numpy as np
import threading
import time
from aiymakerkit import vision
from aiymakerkit import utils
from pycoral.utils.dataset import read_label_file
from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.edgetpu import make_interpreter
from collections import namedtuple

# Create a compatible BBox class for object detection results
BBox = namedtuple('BBox', ['xmin', 'ymin', 'xmax', 'ymax'])

# Create compatible object class for detection results
class DetectedObject:
    def __init__(self, obj, label_map):
        self.score = obj.score
        self.id = obj.id
        self.label = label_map.get(obj.id, str(obj.id))
        # Convert aiymakerkit bbox format to our format
        self.bbox = BBox(
            xmin=obj.bbox[0],
            ymin=obj.bbox[1],
            xmax=obj.bbox[2],
            ymax=obj.bbox[3]
        )

class AISensor:
    def __init__(self):
        self.results = []
        self.ready = False
        self.frame = None
        self.lock = threading.Lock()
        self.last_inference_time = 0
        self.min_inference_interval = 0.1  # Limit to 10 inferences per second max
        
        # Get paths to model and labels
        self.path = self._get_model_path
        model_path = self.path('pascal_model.tflite')
        labels_path = self.path('pascal_model.txt')
        
        # Try to load the model and labels
        try:
            # First try to use Coral TPU
            print("Initializing AI model with Coral TPU acceleration...")
            self.use_tpu = True
            self.interpreter = make_interpreter(model_path)
            self.interpreter.allocate_tensors()
            
            # Get model details
            self.input_size = common.input_size(self.interpreter)
            print(f"Model input size: {self.input_size}")
            
            # Load labels
            self.labels = read_label_file(labels_path)
            print("AI model loaded successfully with TPU acceleration")
            
            # Fallback to aiymakerkit if needed
            self.detector = vision.Detector(model_path)
            
            self.ready = True
        except Exception as e:
            print(f"Error loading AI model with TPU: {e}")
            print("Falling back to CPU inference")
            self.use_tpu = False
            try:
                # Fallback to standard detector
                self.detector = vision.Detector(model_path)
                self.labels = read_label_file(labels_path)
                print("AI model loaded with CPU (no acceleration)")
                self.ready = True
            except Exception as e2:
                print(f"Error loading AI model: {e2}")
                self.detector = None
                self.labels = None
                print("AI functionality will be limited")
    
    def _get_model_path(self, name):
        """Helper to get full path to a model/label file"""
        root = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(root, 'models', name)
    
    def run(self, frame):
        if frame is None or (self.detector is None and not self.use_tpu):
            with self.lock:
                self.results = []
            return
            
        # Throttle inference rate to avoid overloading the system
        current_time = time.time()
        if current_time - self.last_inference_time < self.min_inference_interval:
            return
            
        self.last_inference_time = current_time
            
        try:
            # Save frame for debugging if needed
            self.frame = frame.copy()
            
            compatible_objects = []
            
            # Resize frame for display to reduce lag
            display_frame = cv2.resize(frame, (320, 240))
            
            if self.use_tpu:
                try:
                    # Resize and format the image for the model
                    img = cv2.resize(frame, self.input_size)
                    # Preprocess image for TPU (RGB format)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    common.set_input(self.interpreter, img)
                    
                    # Run inference on TPU
                    start_time = time.time()
                    self.interpreter.invoke()
                    inference_time = time.time() - start_time
                    
                    # Get results
                    results = detect.get_objects(self.interpreter, 0.4)
                    print(f"TPU inference time: {inference_time*1000:.1f}ms, found {len(results)} objects")
                    
                    # Convert results to standard format
                    for obj in results:
                        # Map tensor indices to our DetectedObject format
                        tpu_obj = type('', (), {})()
                        tpu_obj.score = obj.score
                        tpu_obj.id = obj.id
                        tpu_obj.bbox = [
                            obj.bbox.xmin, 
                            obj.bbox.ymin, 
                            obj.bbox.xmax, 
                            obj.bbox.ymax
                        ]
                        compatible_objects.append(DetectedObject(tpu_obj, self.labels))
                        
                        # Draw on display frame
                        xmin = int(obj.bbox.xmin * display_frame.shape[1] / self.input_size[0])
                        ymin = int(obj.bbox.ymin * display_frame.shape[0] / self.input_size[1])
                        xmax = int(obj.bbox.xmax * display_frame.shape[1] / self.input_size[0])
                        ymax = int(obj.bbox.ymax * display_frame.shape[0] / self.input_size[1])
                        
                        label = self.labels.get(obj.id, str(obj.id))
                        cv2.rectangle(display_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                        cv2.putText(display_frame, f"{label}: {obj.score:.2f}", 
                                    (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                    
                    self.frame = display_frame
                    
                except Exception as e:
                    print(f"TPU inference error: {e}, falling back to CPU")
                    # Fallback to regular detector
                    objects = self.detector.get_objects(frame, threshold=0.4)
                    compatible_objects = [DetectedObject(obj, self.labels) for obj in objects]
                    vision.draw_objects(display_frame, objects, self.labels)
                    self.frame = display_frame
            else:
                # Use regular detector
                objects = self.detector.get_objects(frame, threshold=0.4)
                compatible_objects = [DetectedObject(obj, self.labels) for obj in objects]
                vision.draw_objects(display_frame, objects, self.labels)
                self.frame = display_frame
            
            with self.lock:
                self.results = compatible_objects
            
            self.ready = True
        except Exception as e:
            print(f"Error in AI detection: {e}")
            with self.lock:
                self.results = []
    
    def read(self):
        """Return AI detection results"""
        with self.lock:
            return self.results.copy() if self.results else []
    
    def get_frame_with_boxes(self):
        """Return the frame with bounding boxes drawn on it"""
        return self.frame
