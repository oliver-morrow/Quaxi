def get_line_position(vehicle):
    """
    Analyze grayscale sensor data to determine line position
    
    Args:
        vehicle: Vehicle instance with grayscale sensors
        
    Returns:
        List indicating relative line position [left, center, right]
    """
    # Get raw sensor values
    sensor_values = vehicle.get_grayscale_data()
    
    # Handle missing/invalid data
    if not sensor_values or len(sensor_values) == 0:
        sensor_values = [0, 0, 0]  # Default to no line detected
    
    # Process into line position status
    return vehicle.get_line_status(sensor_values)
