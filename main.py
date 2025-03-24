"""
Quaxi - Main Entry Point

This script initializes and runs the Quaxi autonomous vehicle system.
It handles hardware initialization, sensor setup, and autonomous navigation.

Usage:
    python3 main.py                     # Run standard autonomous mode
    python3 main.py --navigate x y      # Navigate to specific coordinates
    python3 main.py --manual            # Run in manual mode
"""

import argparse
import sys
from time import time, sleep

# Import core components
from picarx import Picarx
from DrivingModule import CarController
from Sensors import Sensors
from globals import Zone, State, CURRENT_STATE, CURRENT_ZONE
from ShortTermController import ShortTermController
from nodeTraversal import NodeTraversal
from SignalBrakeLights import signal_left, brake_light, main_light

def parse_arguments():
    parser = argparse.ArgumentParser(description='Quaxi Autonomous Vehicle')
    parser.add_argument('--navigate', nargs=2, type=float, metavar=('X', 'Y'),
                        help='Navigate to specific X,Y coordinates')
    parser.add_argument('--manual', action='store_true',
                        help='Run in manual control mode')
    parser.add_argument('--sound-test', action='store_true',
                        help='Run sound test mode')
    return parser.parse_args()

def setup_hardware():
    print("Initializing hardware...")
    picarx = Picarx()
    car = CarController(picarx)
    sensors = Sensors(picarx)
    
    # Turn on main lights
    main_light(True)
    
    print("Starting sensors...")
    sensors.run()
    
    print("Hardware initialization complete!")
    return picarx, car, sensors

def run_autonomous_mode(car, sensors, short_term):
    print("Starting autonomous mode...")
    short_term.init()
    
    try:
        while True:
            short_term.iteration()
    except KeyboardInterrupt:
        print("Stopping autonomous mode...")
        car.stop()
        main_light(False)

def run_navigation_mode(car, sensors, short_term, dest_x, dest_y):
    print(f"Navigating to coordinates: ({dest_x}, {dest_y})")
    short_term.init()
    
    # Create traversal handler
    traversal = NodeTraversal()
    
    try:
        # Starting from a predefined node - we should ideally detect current position
        traversal.execute_path("PondsideAve.:QuackSt", dest_x, dest_y)
        print("Navigation complete!")
    except KeyboardInterrupt:
        print("Navigation interrupted!")
    finally:
        car.stop()
        main_light(False)

def run_manual_mode(car):
    print("Starting manual control mode...")
    from Sound import manual, main
    # Manual mode uses Sound.py module for controls
    try:
        main()
    except KeyboardInterrupt:
        print("Manual mode stopped")
    finally:
        car.stop()
        main_light(False)

def run_sound_test():
    print("Running sound test...")
    from Sound import main
    main()

def main():
    args = parse_arguments()
    
    if args.sound_test:
        run_sound_test()
        return
    
    # Setup hardware components
    picarx, car, sensors = setup_hardware()
    
    # Create the short-term controller
    short_term = ShortTermController(car, sensors)
    
    try:
        if args.navigate:
            dest_x, dest_y = args.navigate
            run_navigation_mode(car, sensors, short_term, dest_x, dest_y)
        elif args.manual:
            run_manual_mode(car)
        else:
            run_autonomous_mode(car, sensors, short_term)
    finally:
        # Ensure we clean up properly
        print("Shutting down Quaxi...")
        car.stop()
        main_light(False)

if __name__ == "__main__":
    main()