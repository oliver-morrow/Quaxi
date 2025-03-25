import cv2
import threading
import time
from AIRecog import AISensor
import numpy as np

class Sensors:
    def __init__(self, picarx_instance):
        self.px = picarx_instance
        self.ai = AISensor()
        self.cap = None
        self.running = False
        self.camera_thread = None
        self.frame = None
        self.WHITE_THRESHOLD = 700  # Threshold for white line detection
        
    def _camera_thread_function(self):
        """Thread function to continuously capture frames and process them"""
        print("Starting camera thread...")
        
        # Initialize camera
        try:
            self.cap = cv2.VideoCapture(0)  # Try the default camera
            if not self.cap.isOpened():
                print("Failed to open camera #0, trying camera #1...")
                self.cap = cv2.VideoCapture(1)  # Try alternative camera
                
            if not self.cap.isOpened():
                print("Failed to open any camera!")
                return
                
            # Set camera properties for better performance
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 15)  # Lower FPS to reduce CPU load
            
            print("Camera initialized successfully")
            
            while self.running:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("Failed to read frame from camera!")
                    time.sleep(0.1)
                    continue
                    
                # Save frame for processing
                self.frame = frame.copy()
                
                # Run AI detection on a smaller frame to reduce load
                small_frame = cv2.resize(frame, (320, 240))
                self.ai.run(small_frame)
                
                # Optional: Display the frame with detection results
                display_frame = self.ai.get_frame_with_boxes()
                if display_frame is not None:
                    cv2.imshow("AI Detection", display_frame)
                    cv2.waitKey(1)
                    
                time.sleep(0.05)  # Short delay to prevent CPU overuse
                
        except Exception as e:
            print(f"Camera thread error: {e}")
        finally:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
    
    def get_ai_detections(self):
        """Return AI detection results"""
        return self.ai.read()
    
    def get_grayscale_values(self):
        """Get raw grayscale sensor values"""
        return self.px.get_grayscale_data()
    
    def get_line_status(self):
        """Get boolean line status (True if white line detected)"""
        values = self.get_grayscale_values()
        return [val > self.WHITE_THRESHOLD for val in values]
    
    def get_proximity(self):
        """Get distance from ultrasonic sensor with validation"""
        for _ in range(3):  # Try up to 3 times
            distance = self.px.get_distance()
            if distance is not None and 0 < distance < 300:  # Valid range
                return distance
            time.sleep(0.05)  # Short delay before retry
        return None  # Return None if no valid reading
    
    def get_current_frame(self):
        """Return the current camera frame if available"""
        return self.frame
    
    def run(self):
        """Start the camera thread for AI detection"""
        self.running = True
        self.camera_thread = threading.Thread(target=self._camera_thread_function)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        # Wait a moment for camera to initialize
        time.sleep(2)
        print("Sensors ready")

    def stop(self):
        """Stop the camera thread"""
        self.running = False
        if self.camera_thread:
            self.camera_thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

