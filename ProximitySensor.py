def ProximitySensor(px):
    # Try up to 3 times to get a valid reading
    for attempt in range(3):
        try:
            distance = px.get_distance()
            print(f"Raw distance reading: {distance} cm (attempt {attempt+1})")
            
            # Validate the reading
            if distance is not None and distance > 0 and distance < 300:  # Valid range: 0-300cm
                # Valid reading received
                return distance
            else:
                # Invalid reading, print debug info
                print(f"Invalid distance value: {distance}, retrying...")
                # Short delay before retry
                import time
                time.sleep(0.1)
        except Exception as e:
            print(f"Error reading distance sensor: {e}")
    
    # If we reach here, we couldn't get a valid reading after all attempts
    print("Warning: Could not get valid distance reading after multiple attempts")
    return None  # Return None instead of invalid value
