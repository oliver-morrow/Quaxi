from globals import State
from SignalBrakeLights import brake_light
import time

# Simple line following function for forward state
def Forward(car, sensors, iteration_count, distance_value=None):
    """
    Implements forward movement with line following
    
    Args:
        car: CarController instance
        sensors: Sensors instance
        iteration_count: Current iteration count
        distance_value: Optional distance to travel (if specified)
        
    Returns:
        True when movement is complete, False otherwise
    """
    # Read line tracking sensor data
    hardware = sensors.ReadHardware()
    line_data = hardware["lineTracker"]
    
    # Basic obstacle detection with validation
    proximity = hardware["proximity"]
    if proximity is not None and proximity > 0 and proximity < 20:  # 20cm threshold and positive value check
        print(f"Obstacle detected at {proximity} cm - stopping!")
        car.stop()
        brake_light(True)
        time.sleep(0.5)
        brake_light(False)
        return True
    
    # Simple line following logic
    if line_data[0] and not line_data[2]:  # Line on left
        car.turn_left(speed=30, angle=-20)
    elif line_data[2] and not line_data[0]:  # Line on right
        car.turn_right(speed=30, angle=20)
    else:  # Go straight
        car.turn_right(angle=0, speed=30)
    
    # If we're using a distance value, calculate if we've completed
    if distance_value is not None:
        # Simple time-based distance calculation (assumes constant speed)
        # Approximately 30 iterations per second
        if iteration_count > (distance_value / 2):  # Adjust divisor based on speed
            car.stop()
            return True
    
    # Continue moving if no distance specified or not yet reached
    return False
