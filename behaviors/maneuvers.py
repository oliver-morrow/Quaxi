from time import sleep
from peripherals.lighting import activate_left_indicator, activate_right_indicator, set_brake_lights

def perform_wait_sequence(vehicle_controller):
    """Stop and wait at intersections, stop signs"""
    vehicle_controller.drive_forward(0)
    set_brake_lights(True)
    sleep(3)  # Mandatory wait time
    set_brake_lights(False)
    return True

def execute_right_turn(vehicle_controller):
    """Complete sequence for making a right turn"""
    # Signal turn intention
    activate_right_indicator()
    
    # Approach turn at reduced speed
    vehicle_controller.drive_forward(15)
    sleep(0.5)
    
    # Execute turning maneuver
    vehicle_controller.steer_right(speed=20, angle=30)
    sleep(1.5)
    
    # Return to straight driving
    vehicle_controller.steer_right(speed=20, angle=0)
    return True

def execute_left_turn(vehicle_controller):
    """Complete sequence for making a left turn"""
    # Signal turn intention
    activate_left_indicator()
    
    # Prepare for turn
    vehicle_controller.drive_forward(15)
    sleep(0.5)
    
    # Execute turning maneuver
    vehicle_controller.steer_left(speed=20, angle=-30)
    sleep(1.5)
    
    # Return to straight driving
    vehicle_controller.steer_right(speed=20, angle=0)
    return True

def navigate_traffic_circle(vehicle_controller, sensors):
    """Navigate through a traffic circle/roundabout"""
    # Signal and enter
    activate_right_indicator()
    vehicle_controller.drive_forward(20)
    sleep(1)
    
    # Follow the curve of the circle
    vehicle_controller.steer_right(speed=20, angle=15)
    sleep(1)
    
    # Check for exit opportunity
    distance = sensors.get_environmental_data()["obstacle_distance"]
    
    # Exit when appropriate
    if distance > 40:
        activate_right_indicator()
        vehicle_controller.steer_right(speed=20, angle=30)
        sleep(1)
        vehicle_controller.steer_right(speed=20, angle=0)
    
    return True

def execute_parking(vehicle_controller):
    """Execute a complete parking maneuver"""
    # Find parking space
    vehicle_controller.drive_forward(15)
    
    # Signal intention
    activate_right_indicator()
    set_brake_lights(True)
    
    # Position vehicle
    vehicle_controller.drive_forward(15)
    sleep(0.5)
    
    # Begin parking sequence
    vehicle_controller.steer_right(angle=30)
    vehicle_controller.drive_backward(15)
    sleep(1)
    
    # Straighten in parking space
    vehicle_controller.steer_right(angle=0)
    vehicle_controller.drive_backward(15)
    sleep(0.5)
    
    # Final adjustment
    vehicle_controller.drive_forward(10)
    sleep(0.3)
    
    # Secure vehicle
    vehicle_controller.halt()
    set_brake_lights(True)
    return True
