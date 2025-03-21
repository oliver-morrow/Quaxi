import re
import math
from config.constants import State, NEXT_STATE

# Recognizable object categories
OBJECT_CATEGORIES = [
    'duck_regular', 'sign_noentry', 'sign_oneway_left', 
    'sign_oneway_right', 'sign_stop', 'sign_yield'
]

# Camera and movement parameters
MAX_CAMERA_ANGLE = 40
current_camera_angle = 0
normal_driving_speed = 300

def detect_and_track_signs(detection_data, vehicle_controller):
    """Track and follow traffic signs in the camera view"""
    global current_camera_angle
    
    # Check for valid detection data
    if detection_data is None or detection_data.boxes is None:
        return None

    # Find most significant sign in field of view
    best_sign = None
    largest_area = 0
    
    # Analyze all detected objects
    for detection in detection_data.boxes:
        # Skip low-confidence detections
        if detection.conf < 0.70:
            continue
            
        # Find and track largest sign
        area = detection.xywh.w * detection.xywh.h
        if area > largest_area:
            object_type = OBJECT_CATEGORIES[detection.cls]
            if re.search("^sign_.*$", object_type):
                largest_area = area
                best_sign = detection
    
    if best_sign is None:
        return None

    # Calculate camera adjustment to track sign
    adjustment = 0
    if best_sign.xywhn.x > 0.5:
        adjustment = -vehicle_controller.current_speed / 15 - math.exp(-2 * current_camera_angle / MAX_CAMERA_ANGLE)

    # Update camera position with limits
    current_camera_angle = max(min(MAX_CAMERA_ANGLE, current_camera_angle + adjustment), -MAX_CAMERA_ANGLE)
    vehicle_controller.set_camera_angle(current_camera_angle)
    
    return best_sign

def avoid_obstacles(controller, obstacle_distance):
    """Implement obstacle avoidance behavior"""
    # Immediate avoidance for close obstacles
    if obstacle_distance is not None and obstacle_distance < 20:
        controller.steer_right(angle=30, speed=150)
        return True
        
    # Cautious approach for moderate distances
    elif obstacle_distance is not None and obstacle_distance < 40:
        controller.drive_forward(speed=150)
        return True
    
    # No obstacle requiring action
    return False

def drive_forward(controller, sensors):
    """Main driving function with obstacle avoidance and sign recognition"""
    global normal_driving_speed

    # Get current sensor data
    environment = sensors.get_environmental_data()
    
    # Basic lane following
    if environment["lane_position"] == [1, 0, 0]:
        controller.steer_right(speed=0)
    elif environment["lane_position"] == [0, 0, 1]:
        controller.steer_left(speed=0)

    # Process vision data
    vision_data = sensors.get_vision_detections()
    sign = detect_and_track_signs(vision_data, controller)

    # Respond to detected signs
    if sign is not None:
        sign_type = OBJECT_CATEGORIES[sign.cls]
        
        # Actions based on sign type
        if sign_type == 'sign_noentry':
            return 1  # Signal need for state change
        elif sign_type == 'sign_oneway_left':
            if NEXT_STATE == State.TurnR:
                print("Alert: Incorrect direction for one-way route")
            return 1
        elif sign_type == 'sign_oneway_right':
            if NEXT_STATE == State.TurnL:
                print("Alert: Incorrect direction for one-way route")
            return 1
        elif sign_type in ['sign_stop', 'sign_yield']:
            return 0  # Stop/yield but don't change state yet

    # Handle obstacle avoidance
    if avoid_obstacles(controller, environment["obstacle_distance"]):
        return 0

    # Default behavior: drive forward
    controller.steer_right(angle=0, speed=normal_driving_speed)
    return 0
