import cv2
import numpy as np
import threading
import io
import time
from picamera import PiCamera
from perception.vision import ObjectDetector
from sensors.line_detector import get_line_position
from sensors.distance_sensor import measure_distance

def vision_processing_thread(sensor_system):
    """Background thread for continuous visual processing"""
    while sensor_system.is_running:
        start_time = time.time()
        
        # Capture frame
        try:
            stream = io.BytesIO()
            sensor_system.camera.capture(stream, format='jpeg')
            # Convert the picture into a numpy array
            data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            # Convert to an OpenCV image
            frame = cv2.imdecode(data, 1)
            
            # Process with AI
            sensor_system.vision_module.process_frame(frame)
            
            # Reset stream
            stream.seek(0)
            stream.truncate()
            
            # Calculate processing time and sleep if needed
            processing_time = time.time() - start_time
            sleep_time = max(0, 1/24 - processing_time)  # Maintain max 24fps
            if sleep_time > 0:
                time.sleep(sleep_time)
                
            if sensor_system.debug:
                sensor_system.debug.verbose(f"Vision processing cycle: {processing_time:.4f}s, sleep: {sleep_time:.4f}s")
                
        except Exception as e:
            if sensor_system.debug:
                sensor_system.debug.error(f"Vision processing error: {e}")
            time.sleep(0.1)  # Sleep briefly before retrying

class IntegratedSensorSystem:
    """Manages all sensor subsystems on the vehicle"""
    
    def __init__(self, vehicle, debug=None, visualizer=None):
        """Initialize with a reference to the vehicle"""
        self.vehicle = vehicle
        self.debug = debug
        self.is_running = False
        
        # Initialize vision module with visualizer
        self.vision_module = ObjectDetector(debug=debug, visualizer=visualizer)
        
        # Initialize camera
        try:
            self.camera = PiCamera()
            self.camera.resolution = (640, 640)
            self.camera.framerate = 24
            if self.debug:
                self.debug.info("Camera initialized successfully")
        except Exception as e:
            if self.debug:
                self.debug.error(f"Failed to initialize camera: {e}")
            self.camera = None
        
        # Thread for vision processing
        self.vision_thread = None
        
        # Face tracking settings
        self.face_tracking_enabled = False
        self.current_face_position = None
    
    def get_vision_detections(self):
        """Retrieve latest object detection results"""
        return self.vision_module.detections
    
    def get_environmental_data(self):
        """Collect data from all environmental sensors"""
        try:
            if self.debug:
                self.debug.start_timer("sensor_readings")
                
            sensor_readings = {
                "obstacle_distance": measure_distance(self.vehicle),
                "lane_position": get_line_position(self.vehicle)
            }
            
            if self.debug:
                elapsed = self.debug.end_timer("sensor_readings")
                self.debug.verbose(f"Sensor readings collected in {elapsed:.4f}s")
            
            return sensor_readings
            
        except Exception as e:
            if self.debug:
                self.debug.error(f"Error getting sensor data: {e}")
            # Return safe default values
            return {
                "obstacle_distance": None,
                "lane_position": [0, 0, 0]
            }
    
    def analyze_scene(self):
        """Perform general scene analysis on camera input"""
        # Get current frame
        if not self.camera:
            if self.debug:
                self.debug.warning("Camera not available for scene analysis")
            return 0
        
        try:
            if self.debug:
                self.debug.start_timer("scene_analysis")
                
            stream = io.BytesIO()
            self.camera.capture(stream, format='jpeg')
            data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
            current_image = cv2.imdecode(data, 1)
            
            if current_image is None:
                if self.debug:
                    self.debug.warning("Failed to capture image for scene analysis")
                return 0
            
            # Basic image analysis pipeline
            grayscale = cv2.cvtColor(current_image, cv2.COLOR_RGB2GRAY)
            blurred = cv2.GaussianBlur(grayscale, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            
            # Compute scene complexity metric
            edge_density = np.count_nonzero(edges)
            complexity_score = min(100, edge_density / 500)
            
            if self.debug:
                elapsed = self.debug.end_timer("scene_analysis")
                self.debug.debug(f"Scene analysis complete - complexity: {complexity_score:.1f}, time: {elapsed:.4f}s")
            
            return complexity_score
            
        except Exception as e:
            if self.debug:
                self.debug.error(f"Scene analysis error: {e}")
            return 0
    
    def enable_face_tracking(self, enabled=True):
        """Enable or disable face tracking"""
        self.face_tracking_enabled = enabled
        if self.debug:
            self.debug.info(f"Face tracking {'enabled' if enabled else 'disabled'}")
    
    def update_face_tracking(self):
        """Update face tracking if enabled"""
        if not self.face_tracking_enabled:
            return
            
        # Implementation would use OpenCV face detection
        # This is a placeholder for the actual implementation
        pass
    
    def begin_monitoring(self):
        """Start all sensor monitoring systems"""
        if self.is_running:
            return
            
        self.is_running = True
        
        if self.debug:
            self.debug.info("Starting sensor monitoring systems")
            
        # Start vision processing thread if camera is available
        if self.camera:
            self.vision_thread = threading.Thread(target=vision_processing_thread, args=(self,))
            self.vision_thread.daemon = True
            self.vision_thread.start()
            
            if self.debug:
                self.debug.info("Vision processing thread started")
    
    def end_monitoring(self):
        """Properly shut down all sensor systems"""
        if not self.is_running:
            return
            
        self.is_running = False
        
        if self.debug:
            self.debug.info("Stopping sensor monitoring systems")
            
        # Stop vision thread
        if hasattr(self, 'vision_thread') and self.vision_thread and self.vision_thread.is_alive():
            self.vision_thread.join(timeout=1.0)
            
        # Stop vision module
        if hasattr(self, 'vision_module') and hasattr(self.vision_module, 'stop'):
            self.vision_module.stop()
            
        # Close camera
        if self.camera:
            self.camera.close()
            self.camera = None
            
        if self.debug:
            self.debug.info("Sensor monitoring systems stopped")
