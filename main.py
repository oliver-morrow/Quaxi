#!/usr/bin/env python3
import time
import signal
import sys
import argparse
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
from utils.debug import QuaxiDebug, DebugLevel
from utils.visualization import CameraVisualizer

# Global debug and visualization objects
debug = None
visualizer = None

# Clean shutdown handler
def signal_handler(sig, frame):
    if debug:
        debug.info('Shutting down systems...')
    else:
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
    
    if visualizer:
        visualizer.stop()
    
    if debug:
        debug.info('Shutdown complete')
    else:
        print('Shutdown complete')
    
    sys.exit(0)

def parse_arguments():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description='Quaxi Autonomous Vehicle Control')
    parser.add_argument('--debug', choices=['none', 'error', 'warning', 'info', 'debug', 'verbose'],
                      default='info', help='Set debug level')
    parser.add_argument('--log', action='store_true', help='Enable file logging')
    parser.add_argument('--visualize', action='store_true', help='Enable camera visualization')
    parser.add_argument('--test-mode', action='store_true', help='Run in test mode without activating motors')
    
    return parser.parse_args()

def main():
    # Parse command line arguments
    args = parse_arguments()
    
    # Initialize debug system
    global debug
    debug_level = getattr(DebugLevel, args.debug.upper())
    debug = QuaxiDebug(level=debug_level, log_to_file=args.log)
    
    # Initialize visualization if requested
    global visualizer
    if args.visualize:
        visualizer = CameraVisualizer()
        debug.info("Camera visualization enabled")
    
    # Register signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    debug.info("Initializing Quaxi autonomous vehicle systems...")
    
    # Initialize vehicle hardware
    vehicle = RobotVehicle(debug=debug)
    vehicle.reset_systems()
    
    # Initialize controller
    controller = VehicleController(vehicle, debug=debug)
    
    # Initialize sensor systems with visualization if enabled
    sensors = IntegratedSensorSystem(vehicle, debug=debug, visualizer=visualizer)
    sensors.begin_monitoring()
    
    # Initialize audio system
    audio = VehicleAudioSystem()
    audio.speak_message("Vehicle systems online")
    
    # Turn on headlights
    set_headlights(True)
    
    # Warm-up period
    debug.info("Systems initialized. Starting in 3 seconds...")
    time.sleep(3)
    
    # Main driving loop
    try:
        global CURRENT_STATE, NEXT_STATE
        CURRENT_STATE = State.WAIT
        
        while True:
            # Update face tracking if enabled
            sensors.update_face_tracking()
            
            # Log current state
            debug.debug(f"Current state: {CURRENT_STATE.name}, Next state: {NEXT_STATE.name}")
            
            # State machine for vehicle behavior
            if CURRENT_STATE == State.WAIT:
                debug.info("Executing wait sequence")
                perform_wait_sequence(controller)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.DRIVE_FORWARD:
                result = drive_forward(controller, sensors)
                if result == 1:
                    debug.info(f"Changing state from DRIVE_FORWARD to {NEXT_STATE.name}")
                    CURRENT_STATE = NEXT_STATE
                
            elif CURRENT_STATE == State.TURN_RIGHT:
                debug.info("Executing right turn")
                execute_right_turn(controller)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.TURN_LEFT:
                debug.info("Executing left turn")
                execute_left_turn(controller)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.ROUNDABOUT:
                debug.info("Navigating traffic circle")
                navigate_traffic_circle(controller, sensors)
                CURRENT_STATE = State.DRIVE_FORWARD
                
            elif CURRENT_STATE == State.PARKING:
                debug.info("Executing parking maneuver")
                execute_parking(controller)
                # Stay in PARKING state until manually changed
            
            # Small delay to prevent CPU overuse
            time.sleep(0.05)
            
    except Exception as e:
        debug.error(f"Error in main control loop: {e}")
        signal_handler(None, None)

if __name__ == "__main__":
    main()