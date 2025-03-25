import time
import cv2
import numpy as np
from picarx import Picarx
from SignalBrakeLights import brake_light, signal_left, signal_right, main_light
from ProximitySensor import ProximitySensor

WHITE_THRESHOLD = 700  # Threshold for detecting white lines
DRIVING_SPEED = 20     # Default driving speed
TURN_WAIT_TIME = 2     # Time to wait at intersections

class QuaxiControl:
    def __init__(self):
        # Initialize hardware
        self.px = Picarx()
        self.cap = None
        self.camera_enabled = False
        
        # Configure settings
        self.neutral_angle = -10  # Slight offset adjustment if needed
        main_light(True)          # Turn on headlights
        
        # Position camera
        self.px.set_cam_pan_angle(-10)  # Slight pan left
        self.px.set_cam_tilt_angle(-15)  # Tilt down to see the road
        
    def init_camera(self):
        """Initialize camera if needed for lane detection"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Failed to open camera #0, trying camera #1...")
                self.cap = cv2.VideoCapture(1)
                
            if self.cap.isOpened():
                self.camera_enabled = True
                print("Camera initialized successfully")
                return True
            else:
                print("Failed to open any camera!")
                return False
        except Exception as e:
            print(f"Camera initialization error: {e}")
            return False
            
    def get_grayscale_data(self):
        """Get grayscale sensor values"""
        return self.px.get_grayscale_data()
        
    def get_line_status(self):
        """Get boolean line status (True if white line detected)"""
        values = self.get_grayscale_data()
        return [val > WHITE_THRESHOLD for val in values]
        
    def detect_stop_line(self):
        """Check for white stop line using grayscale sensors"""
        sensor_values = self.get_grayscale_data()
        print("Grayscale sensor readings:", sensor_values)
        
        # If all sensors detect a high value (likely a white stop line)
        if all(value > WHITE_THRESHOLD for value in sensor_values):
            print("Stop line detected! Stopping the car.")
            return True
        return False
        
    def adjust_direction(self):
        """Adjust the car's direction based on grayscale sensor values"""
        sensor_values = self.get_grayscale_data()
        left_sensor = sensor_values[0]
        middle_sensor = sensor_values[1]
        right_sensor = sensor_values[2]

        print(f"Grayscale readings: L={left_sensor}, M={middle_sensor}, R={right_sensor}")

        if left_sensor > WHITE_THRESHOLD and right_sensor < WHITE_THRESHOLD:
            print("Line on left, turning right.")
            self.px.set_dir_servo_angle(20)  # Turn right
        elif right_sensor > WHITE_THRESHOLD and left_sensor < WHITE_THRESHOLD:
            print("Line on right, turning left.")
            self.px.set_dir_servo_angle(-20)  # Turn left
        elif middle_sensor > WHITE_THRESHOLD:
            print("Following middle line, going straight.")
            self.px.set_dir_servo_angle(self.neutral_angle)  # Go straight
        else:
            # Keep current angle if no strong detection
            print("Maintaining current direction.")
        
    def wait_for_user_input(self):
        """Wait for the user to input a direction for the car"""
        while True:
            print("\nPlease choose a direction:")
            print("1: Turn Left")
            print("2: Turn Right")
            print("3: Move Forward")
            user_input = input("Enter your choice (1/2/3): ").strip()

            if user_input == "1":
                self.turn_left()
                break
            elif user_input == "2":
                self.turn_right()
                break
            elif user_input == "3":
                self.move_forward()
                break
            else:
                print("Invalid choice, please try again.")
                
    def turn_left(self):
        """Execute a left turn at an intersection"""
        print("Turning left.")
        signal_left(times=3, interval=0.3)  # Signal left turn
        self.px.set_dir_servo_angle(-30)  # Adjust angle for left turn
        self.px.forward(DRIVING_SPEED)
        time.sleep(1.5)  # Time to complete turn
        
        # Use sensor feedback to complete the turn
        attempts = 0
        max_attempts = 30  # Avoid infinite loop
        
        while attempts < max_attempts:
            sensor_values = self.get_grayscale_data()
            right_sensor = sensor_values[2]
            middle_sensor = sensor_values[1]
            
            if middle_sensor > WHITE_THRESHOLD:  # Middle sensor detects line
                print("Center line detected! Turn complete.")
                break
                
            time.sleep(0.1)
            attempts += 1
            
        self.px.set_dir_servo_angle(self.neutral_angle)  # Reset steering
    
    def turn_right(self):
        """Execute a right turn at an intersection"""
        print("Turning right.")
        signal_right(times=3, interval=0.3)  # Signal right turn
        self.px.set_dir_servo_angle(30)  # Adjust angle for right turn
        self.px.forward(DRIVING_SPEED)
        time.sleep(1.5)  # Time to complete turn
        
        # Use sensor feedback to complete the turn
        attempts = 0
        max_attempts = 30  # Avoid infinite loop
        
        while attempts < max_attempts:
            sensor_values = self.get_grayscale_data()
            left_sensor = sensor_values[0]
            middle_sensor = sensor_values[1]
            
            if middle_sensor > WHITE_THRESHOLD:  # Middle sensor detects line
                print("Center line detected! Turn complete.")
                break
                
            time.sleep(0.1)
            attempts += 1
            
        self.px.set_dir_servo_angle(self.neutral_angle)  # Reset steering
    
    def move_forward(self):
        """Move forward at an intersection"""
        print("Moving forward.")
        self.px.set_dir_servo_angle(self.neutral_angle)
        self.px.forward(DRIVING_SPEED)
        time.sleep(1.0)  # Time to clear intersection
    
    def check_proximity(self):
        """Check for obstacles using ultrasonic sensor"""
        distance = ProximitySensor(self.px)
        if distance is not None and distance < 30:  # Objects closer than 30cm
            print(f"Obstacle detected at {distance}cm! Stopping.")
            return True
        return False
        
    def run(self):
        """Main control loop"""
        try:
            # Initialize camera if available
            self.init_camera()
            
            print("Starting Quaxi control. Press Ctrl+C to stop.")
            self.px.forward(DRIVING_SPEED)
            
            while True:
                # Check for obstacles first
                if self.check_proximity():
                    self.px.stop()
                    brake_light(True)
                    time.sleep(1)  # Wait briefly
                    brake_light(False)
                    continue  # Skip to next iteration after obstacle detection
                
                # Check for stop line
                if self.detect_stop_line():
                    self.px.stop()
                    brake_light(True)
                    time.sleep(TURN_WAIT_TIME)
                    brake_light(False)
                    
                    # Ask for user direction
                    self.wait_for_user_input()
                else:
                    # Regular line following using grayscale sensors
                    self.adjust_direction()
                
                # Short delay to prevent overloading CPU
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping Quaxi control")
        finally:
            self.px.stop()
            main_light(False)
            if self.cap is not None:
                self.cap.release()

if __name__ == "__main__":
    controller = QuaxiControl()
    controller.run()
