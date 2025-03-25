"""
Quaxi - Autonomous Driving with Navigation

This script provides:
1. Line following using grayscale sensors
2. Map navigation using A* pathfinding
3. Object detection using AI camera
4. Manual start/stop control via keyboard
5. Ultrasonic obstacle avoidance
"""

import time
import cv2
import threading
import sys
import select
from picarx import Picarx
from DrivingModule import CarController
from Sensors import Sensors
from SignalBrakeLights import signal_left, brake_light, main_light
from enhanced_line_follower import EnhancedLineFollower
from pathfinding import PathFinder
# from Sound import music

# Constants
MAX_ANGLE = 35
DRIVING_SPEED = 25
TIME_TO_TURN = 20
OBSTACLE_DISTANCE_THRESHOLD = 30  # Stop if obstacle is closer than this (in cm)
PROXIMITY_CHECK_INTERVAL = 0.1  # Check proximity every 0.1 seconds

class ManualControl:
    def __init__(self):
        self.running = False
        self.command_thread = None
        self.stop_event = threading.Event()
        self.navigation_mode = False
    
    def start_listening(self):
        """Start listening for keyboard commands in a separate thread"""
        self.command_thread = threading.Thread(target=self._command_listener)
        self.command_thread.daemon = True
        self.command_thread.start()
    
    def _command_listener(self):
        """Thread function to listen for keyboard input"""
        print("\n=== MANUAL CONTROL ===")
        print("Type 'start' to start the robot in line following mode")
        print("Type 'stop' to stop the robot")
        print("Type 'nav [destination]' to navigate to a destination")
        print("Type 'exit' to exit the program\n")
        
        while not self.stop_event.is_set():
            # Use select to check if input is available (non-blocking)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                command = sys.stdin.readline().strip().lower()
                
                if command == 'start':
                    print("Starting robot in line following mode...")
                    self.running = True
                    self.navigation_mode = False
                elif command.startswith('nav '):
                    print("Starting navigation mode...")
                    self.running = True
                    self.navigation_mode = True
                    # Extract destination
                    try:
                        # Format: "nav x,y" where x,y are coordinates
                        coords = command.split(' ')[1].split(',')
                        if len(coords) == 2:
                            dest_x = int(coords[0])
                            dest_y = int(coords[1])
                            print(f"Setting destination to ({dest_x}, {dest_y})")
                            return (dest_x, dest_y)
                    except:
                        print("Invalid destination format. Use 'nav x,y'")
                elif command == 'stop':
                    print("Stopping robot...")
                    self.running = False
                elif command == 'exit':
                    print("Exiting program...")
                    self.running = False
                    self.stop_event.set()
                    sys.exit(0)
    
    def stop_listening(self):
        """Stop the command listener thread"""
        self.stop_event.set()
        if self.command_thread:
            self.command_thread.join(timeout=1.0)

def setup_hardware():
    print("Initializing hardware...")
    picarx = Picarx()
    car = CarController(picarx)
    sensors = Sensors(picarx)
    
    # Use EnhancedLineFollower instead of LineFollower
    line_follower = EnhancedLineFollower()
    
    # Set up navigation
    pathfinder = PathFinder(car)
    
    # Position camera to face forward at ideal angle
    print("Adjusting camera position...")
    picarx.set_cam_pan_angle(0)  # Center horizontally
    picarx.set_cam_tilt_angle(-15)  # Tilt down slightly to see the road
    
    # Turn on main lights
    main_light(True)
    # music.sound_play_threading('../sounds/startup.wav')
    
    print("Starting sensors...")
    sensors.run()
    time.sleep(3)  # Allow time for camera to initialize
    
    print("Hardware initialization complete!")
    return picarx, car, sensors, line_follower, pathfinder

def check_obstacles(ai_data, car):
    """Check for obstacles or signs detected by AI"""
    if not ai_data:
        return False
        
    for obj in ai_data:
        # Skip objects with low confidence
        if obj.score < 0.6:
            continue
            
        # Print detection info for debugging
        print(f"Detected: {obj.label} (confidence: {obj.score:.2f})")
            
        # Check for signs that require stopping
        if obj.label in ['sign_stop', 'sign_yield']:
            print(f"Detected {obj.label}, stopping...")
            car.stop()
            brake_light(True)
            # Play sound effect for stop
            # music.sound_play_threading('../sounds/stop.wav')
            time.sleep(2)  # Stop briefly
            brake_light(False)
            return True
            
        # Duck detection (treat as obstacle)
        elif 'duck' in obj.label:
            # Calculate position and size of duck
            width = obj.bbox.xmax - obj.bbox.xmin
            height = obj.bbox.ymax - obj.bbox.ymin
            area = width * height
            
            # If duck is large enough (close), stop
            if area > 0.05:  # Threshold may need adjustment
                print(f"Detected {obj.label} obstacle, stopping...")
                car.stop()
                brake_light(True)
                # music.sound_play_threading('../sounds/horn.wav')
                time.sleep(1)
                brake_light(False)
                return True
                
    return False

def check_ultrasonic(picarx):
    """Check ultrasonic sensor for obstacles"""
    distance = picarx.get_distance()
    # Only consider valid positive distance readings
    if distance is not None and distance > 0 and distance < OBSTACLE_DISTANCE_THRESHOLD:
        print(f"Obstacle detected at {distance:.1f} cm - stopping!")
        return True
    return False

def main():
    # Add debug parameter
    debug_mode = "--debug" in sys.argv
    
    picarx, car, sensors, line_follower, pathfinder = setup_hardware()
    
    # Initialize manual control
    manual_control = ManualControl()
    manual_control.start_listening()
    
    print("Robot initialized and ready!")
    print("Waiting for 'start' command...")
    
    # Timing variables for proximity sensor checking
    last_proximity_check = time.time()
    last_debug_print = time.time()
    
    # Set starting location
    current_location = "PondsideAve.:QuackSt"
    
    try:
        while True:
            # Debug output
            if debug_mode and time.time() - last_debug_print > 1.0:
                hardware = sensors.ReadHardware()
                print(f"Proximity: {hardware['proximity']}, Line: {hardware['lineTracker']}")
                ai_data = sensors.ReadAI()
                if ai_data:
                    for obj in ai_data:
                        print(f"AI: {obj.label} ({obj.score:.2f})")
                last_debug_print = time.time()
                
            # Only perform operations if the robot is running
            if manual_control.running:
                # Check if we're in navigation mode
                if manual_control.navigation_mode:
                    # Get destination from manual control
                    destination = manual_control.get_navigation_destination()
                    if destination:
                        dest_x, dest_y = destination
                        # Start navigation
                        pathfinder.navigate_to(current_location, dest_x, dest_y)
                        # Update current location
                        current_location = pathfinder.current_node
                        # Reset to line following mode
                        manual_control.navigation_mode = False
                
                # Regular line following mode
                else:
                    # Check ultrasonic sensor periodically (more efficient than every loop)
                    current_time = time.time()
                    if current_time - last_proximity_check >= PROXIMITY_CHECK_INTERVAL:
                        obstacle_detected_by_ultrasonic = check_ultrasonic(picarx)
                        last_proximity_check = current_time
                        
                        if obstacle_detected_by_ultrasonic:
                            car.stop()
                            brake_light(True)
                            # music.sound_play_threading('../sounds/brake.wav')
                            time.sleep(0.5)  # Brief stop
                            brake_light(False)
                            continue  # Skip to next iteration
                    
                    # Read sensor data
                    hardware = sensors.ReadHardware()
                    line_data = hardware["lineTracker"]
                    
                    # Check AI data for obstacles or signs
                    ai_data = sensors.ReadAI()
                    obstacle_detected = check_obstacles(ai_data, car)
                    
                    # Only follow line if no obstacles detected
                    if not obstacle_detected:
                        # Use the LineFollower class from simple_line_follower.py
                        steering_angle = line_follower.follow_line(line_data)
                        car.turn_right(angle=steering_angle, speed=DRIVING_SPEED)
            else:
                # Robot is stopped - ensure motors are off
                car.stop()
                
            time.sleep(0.05)  # Short delay to prevent CPU overuse
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        # Clean up
        car.stop()
        main_light(False)
        sensors.stop()
        manual_control.stop_listening()
        print("Program terminated")

if __name__ == "__main__":
    main()