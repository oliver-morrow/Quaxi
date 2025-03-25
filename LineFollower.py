from picarx import Picarx
import time

class LineFollower:
    def __init__(self, picarx_instance=None):
        self.px = picarx_instance if picarx_instance else Picarx()
        self.WHITE_THRESHOLD = 700  # Adjust based on testing
        self.NEUTRAL_ANGLE = -10  # Slight offset to adjust for any physical bias
        self.running = False
        self.detected_stop_line = False
        self.has_user_input = False
        self.user_choice = None
        
    def get_line_status(self):
        """Get status of grayscale sensors - returns [left, center, right]."""
        sensor_values = self.px.get_grayscale_data()
        return [value > self.WHITE_THRESHOLD for value in sensor_values]
    
    def adjust_direction(self):
        """Adjust the car's direction based on grayscale sensor values."""
        sensor_values = self.px.get_grayscale_data()
        left_sensor = sensor_values[0]
        center_sensor = sensor_values[1]
        right_sensor = sensor_values[2]
        
        print(f"Grayscale readings: L={left_sensor}, C={center_sensor}, R={right_sensor}")
        
        if left_sensor > 200 and right_sensor < 200:
            print("Line on left, turning right")
            self.px.set_dir_servo_angle(20)  # Turn right
        elif right_sensor > 200 and left_sensor < 200:
            print("Line on right, turning left")
            self.px.set_dir_servo_angle(-20)  # Turn left
        elif center_sensor > 200:
            print("Line in center, going straight")
            self.px.set_dir_servo_angle(self.NEUTRAL_ANGLE)  # Go straight
        else:
            # Keep current angle if no line detected
            pass
    
    def detect_stop_line(self):
        """Check for white stop line using grayscale sensors."""
        sensor_values = self.px.get_grayscale_data()
        
        # If all sensors detect a high value (likely a white stop line)
        if all(value > self.WHITE_THRESHOLD for value in sensor_values):
            print("STOP LINE DETECTED!")
            self.detected_stop_line = True
            return True
        return False
    
    def handle_stop_line(self):
        """Handle stop line detection."""
        self.px.stop()
        self.px.brake_lights_on()
        print("Stopped at line. Waiting for decision...")
        time.sleep(2)  # Pause for a moment
        self.px.brake_lights_off()
        
        # Implement stop line behavior
        # Could be user input, AI decision, or timer
        # For now, just wait for a few seconds and continue
        time.sleep(3)
        self.detected_stop_line = False
    
    def handle_intersection(self, direction='forward'):
        """Handle the intersection by turning in specified direction."""
        if direction == 'left':
            self.px.turn_signal_left_on()
            print("Turning left at intersection")
            self.px.set_dir_servo_angle(-30)
            self.px.forward(15)
            time.sleep(1.5)  # Time needed to complete turn
            self.px.turn_signal_left_off()
        elif direction == 'right':
            self.px.turn_signal_right_on()
            print("Turning right at intersection")
            self.px.set_dir_servo_angle(30)
            self.px.forward(15)
            time.sleep(1.5)  # Time needed to complete turn
            self.px.turn_signal_right_off()
        else:  # forward
            print("Going straight through intersection")
            self.px.set_dir_servo_angle(self.NEUTRAL_ANGLE)
            self.px.forward(20)
            time.sleep(1.0)
    
    def start(self, speed=20):
        """Start line following."""
        self.running = True
        self.px.forward(speed)
        
        try:
            while self.running:
                # Check for stop line first
                if self.detect_stop_line():
                    self.handle_stop_line()
                    # After handling the stop line, handle the intersection
                    self.handle_intersection('forward')  # Default direction
                else:
                    # No stop line, just adjust direction based on line position
                    self.adjust_direction()
                
                time.sleep(0.05)  # Small delay to prevent overloading CPU
                
        except KeyboardInterrupt:
            print("Line following interrupted")
            self.stop()
    
    def stop(self):
        """Stop line following."""
        self.running = False
        self.px.stop()

    def process_user_input(self, choice):
        """Process user input for intersection navigation."""
        self.user_choice = choice
        self.has_user_input = True
        
        if choice == "1":  # Turn left
            self.handle_intersection('left')
        elif choice == "2":  # Turn right
            self.handle_intersection('right')
        elif choice == "3":  # Go straight
            self.handle_intersection('forward')
        else:
            print("Invalid choice, going straight")
            self.handle_intersection('forward')


if __name__ == "__main__":
    try:
        follower = LineFollower()
        print("Starting line following, press Ctrl+C to stop...")
        follower.start()
    except KeyboardInterrupt:
        print("Stopping line follower...")
    finally:
        follower.stop()
