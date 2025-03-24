from time import time, sleep
from globals import Zone, State, CURRENT_STATE, CURRENT_ZONE
from Forward import Forward

class ShortTermController:
    def __init__(self, car, sensors):
        self.car = car
        self.sensors = sensors
        self.currentPosition = None
        self.nextNode = None
        self.stateStartIteration = 0
        self.relativeIteration = 0
        self.value = 0
        self.stateTransition = False
        self.CURRENT_STATE = State.Wait
        
    def Wait(self):
        self.car.turn_right(0, 0)
        return True
        
    def TurnR(self):
        # check what angle to put servo
        self.car.turn_right()
        return 1
        
    def TurnL(self):
        # check what angle to put servo
        self.car.turn_left()
        return 1
        
    def RoundAbout(self):
        # more complex logic
        return
        
    def Park(self):
        # depends
        return
        
    def iteration(self):
        startTime = time()
        
        self.relativeIteration += 1
        if self.CURRENT_STATE == State.Wait:
            self.stateTransition = self.Wait()
        elif self.CURRENT_STATE == State.Forward:
            self.stateTransition = Forward(self.car, self.sensors, self.relativeIteration, self.value)
        elif self.CURRENT_STATE == State.TurnR:
            self.stateTransition = self.TurnR()
        elif self.CURRENT_STATE == State.TurnL:
            self.stateTransition = self.TurnL()
        elif self.CURRENT_STATE == State.RoundAbout:
            self.stateTransition = self.RoundAbout()
        elif self.CURRENT_STATE == State.Park:
            self.stateTransition = self.Park()
        else:
            print("Unknown State")
        
        endTime = time()
        
        if endTime - startTime < 1/120:
            sleep(1/120 - (endTime - startTime))
        return
    
    def init(self):
        global CURRENT_STATE
        self.sensors.run()
        self.CURRENT_STATE = State.Forward
