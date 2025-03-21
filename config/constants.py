# System constants and state definitions (previously globals.py)
from enum import Enum

class Zone(Enum):
    """Vehicle operational zones"""
    UNDEFINED = 0
    PEDESTRIAN = 1
    HAZARD = 2
    SCHOOL = 3
    RESIDENTIAL = 4
    COMMERCIAL = 5
    INDUSTRIAL = 6

class State(Enum):
    """Vehicle operational states"""
    WAIT = 0
    DRIVE_FORWARD = 1
    TURN_RIGHT = 2
    TURN_LEFT = 3
    ROUNDABOUT = 4
    PARKING = 5

class WayPoint:
    """Navigation waypoint representation"""
    def __init__(self, name=None, coordinates=None):
        self.name = name
        self.coordinates = coordinates or (0, 0)
        self.visited = False

# Global state tracking
CURRENT_ZONE = Zone.UNDEFINED
CURRENT_STATE = State.WAIT
NEXT_STATE = State.WAIT
