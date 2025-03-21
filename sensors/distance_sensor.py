def measure_distance(vehicle):
    """
    Measure distance to obstacles using ultrasonic sensor
    
    Args:
        vehicle: Vehicle instance with ultrasonic sensor
        
    Returns:
        Distance in centimeters, or None if measurement failed
    """
    distance = vehicle.get_distance()
    
    # Basic validation of reading
    if distance is not None and distance > 0:
        return min(distance, 300)  # Cap at 300cm for reliability
    return None
