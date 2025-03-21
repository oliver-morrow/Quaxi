#!/usr/bin/env python3
import time
import signal
import sys
from config.constants import State, CURRENT_STATE, NEXT_STATE
from vehicle.core import RobotVehicle
from vehicle.controller import VehicleController
from sensors.system import IntegratedSensorSystem
from peripherals.audio import VehicleAudioSystem
from peripherals.lighting import set_headlights, set_brake_lights
from navigation.driving import drive_forward
from behaviors.maneuvers import (
    perform_wait_sequence,
    execute_right_turn,
    execute_left_turn,
    navigate_traffic_circle,
    execute_parking
)

# Clean shutdown handler
def signal_handler(sig, frame):
    print('Shutting down systems...')
    # Cleanup procedures
    set_headlights(False)
    set_brake_lights(False)
    if 'controller' in globals():
        controller.halt()
    if 'sensors' in globals() and hasattr(sensors, 'end_monitoring'):
        sensors.end_monitoring()
    if 'audio' in globals() and hasattr(audio, 'music_player'):
        audio.music_player.music_stop()
    
    print('Shutdown complete')
    sys.exit(0)

def main():
    # Register signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Initializing Quaxi autonomous vehicle systems...")
    
    # Initialize vehicle hardware
    vehicle = RobotVehicle()
    vehicle.reset_systems()
    
    # Initialize controller
    controller = VehicleController(vehicle)
    
    # Initialize sensor systems
    sensors = IntegratedSensorSystem(vehicle)
    sensors.begin_monitoring()
    
    # Enable face tracking for demo purposes (optional)
    # sensors.enable_face_tracking(True)
    
    # Initialize audio system
    audio = VehicleAudioSystem()
    audio.speak_message("Vehicle systems online")
    
    # Turn on headlights
    set_headlights(True)
    
    # Warm-up period
    print("Systems initialized. Starting in 3 seconds...")
    time.sleep(3)
    
    # Main driving loop
    try:
        global CURRENT_STATE, NEXT_STATE
        CURRENT_STATE = State.WAIT
        
        while True:
            # Update face tracking if enabled
            sensors.update_face_tracking()
            
            # State machine for vehicle behavior
            if CURRENT_STATE == State.WAIT:
                perform_wait_sequence(controller)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.DRIVE_FORWARD:
                result = drive_forward(controller, sensors)
                if result == 1:
                    CURRENT_STATE = NEXT_STATE
                
            elif CURRENT_STATE == State.TURN_RIGHT:
                execute_right_turn(controller)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.TURN_LEFT:
                execute_left_turn(controller)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.ROUNDABOUT:
                navigate_traffic_circle(controller, sensors)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.PARKING:
                execute_parking(controller)
                # Stay in PARKING state until manually changed
            
            # Small delay to prevent CPU overuse
            time.sleep(0.05)
            
    except Exception as e:
        print(f"Error in main control loop: {e}")
        signal_handler(None, None)

if __name__ == "__main__":
    main()