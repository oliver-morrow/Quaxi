import os
from aiymakerkit import vision
from aiymakerkit import utils
from pycoral.utils.dataset import read_label_file

class AISensor:
    def __init__(self):
        self.results = None
        self.ready = False
        
        # Get paths to model and labels
        root_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(root_dir, "models", "efficientdet-lite-road-signs.tflite")
        labels_path = os.path.join(root_dir, "models", "road-signs-labels.txt")
        
        # Try to load the model and labels
        try:
            self.detector = vision.Detector(model_path)
            self.labels = read_label_file(labels_path)
            print("AI model loaded successfully")
        except Exception as e:
            print(f"Error loading AI model: {e}")
            self.detector = None
            self.labels = None
            print("AI functionality will be limited")
    
    def run(self, frame):
        if self.detector is not None:
            try:
                self.results = self.detector.get_objects(frame, threshold=0.4)
                self.ready = True
            except Exception as e:
                print(f"Error in AI detection: {e}")
                self.results = []
        else:
            self.results = []
    
    def read(self):
        return self.results if self.results else []
