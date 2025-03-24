import cv2
import numpy as np
import threading
import time
import io
from picamera import PiCamera

class CameraVisualizer:
    """Real-time visualization of camera feed with object detection overlay"""
    
    def __init__(self, width=640, height=640, fps=24):
        """Initialize the camera visualization system"""
        self.width = width
        self.height = height
        self.fps = fps
        self.running = False
        self.display_thread = None
        self.camera = None
        self.current_frame = None
        self.detection_results = None
        self.class_labels = None
        
        # Display window name
        self.window_name = "Quaxi Vision System"
        
        # Colors for different object classes (BGR format)
        self.colors = [
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 0, 0),    # Blue
            (255, 255, 0),  # Cyan
            (0, 255, 255),  # Yellow
            (255, 0, 255)   # Magenta
        ]
    
    def start(self, class_labels=None):
        """Start the visualization system"""
        if self.running:
            return
        
        self.class_labels = class_labels or []
        self.running = True
        
        # Initialize camera
        self.camera = PiCamera()
        self.camera.resolution = (self.width, self.height)
        self.camera.framerate = self.fps
        time.sleep(2)  # Allow camera to warm up
        
        # Create and start display thread
        self.display_thread = threading.Thread(target=self._display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()
        
        # Create display window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
    
    def stop(self):
        """Stop the visualization system"""
        self.running = False
        if self.display_thread and self.display_thread.is_alive():
            self.display_thread.join(timeout=1.0)
        
        if self.camera:
            self.camera.close()
        
        cv2.destroyAllWindows()
    
    def update_detections(self, detection_results):
        """Update the current detection results to be displayed"""
        self.detection_results = detection_results
    
    def _display_loop(self):
        """Main loop for capturing and displaying frames"""
        while self.running:
            # Capture frame
            stream = io.BytesIO()
            self.camera.capture(stream, format='jpeg')
            data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            frame = cv2.imdecode(data, 1)
            
            # Store current frame
            self.current_frame = frame.copy()
            
            # Draw detections if available
            if self.detection_results is not None:
                self._draw_detections(frame)
            
            # Add debug info
            self._add_debug_info(frame)
            
            # Display the frame
            cv2.imshow(self.window_name, frame)
            
            # Check for key press (q to quit)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
    
    def _draw_detections(self, frame):
        """Draw detection boxes and labels on the frame"""
        if hasattr(self.detection_results, 'boxes'):
            for i, box in enumerate(self.detection_results.boxes):
                # Extract box data
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                # Skip low confidence detections
                if conf < 0.4:
                    continue
                
                # Determine color based on class
                color = self.colors[cls_id % len(self.colors)]
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # Draw label
                label = f"{self.class_labels[cls_id] if cls_id < len(self.class_labels) else 'Unknown'}: {conf:.2f}"
                cv2.rectangle(frame, (x1, y1-20), (x1+len(label)*8, y1), color, -1)
                cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def _add_debug_info(self, frame):
        """Add debug information to the frame"""
        # Add timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, timestamp, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add FPS counter
        fps = self.camera.framerate
        cv2.putText(frame, f"FPS: {fps}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def get_current_frame(self):
        """Get the current camera frame (without overlays)"""
        return self.current_frame
