from time import time,sleep
from Sensors import Sensors
from DrivingModule import CarController
from picarx import Picarx
from globals import Zone,State,Node,CURRENT_STATE,CURRENT_ZONE

from Forward import Forward


currentPosition=None
nextNode=None
stateStartIteration=0
relativeIteration=0
value=0
picarx=Picarx()
car=CarController(picarx)
sensors=Sensors(picarx)

stateTransition=False

def Wait():
  car.turn_right(0,0)
  return True
def TurnR():
  # check what angle to put servo
  car.turn_right()
  return 1
def TurnL():
  # check what angle to put servo
  car.turn_left()
  return 1
def RoundAbout():
  # more complex logic
  return
def Park():
  # depends
  return
 
def iteration():
  global stateTransition
  global CURRENT_STATE
  global relativeIteration
  startTime=time()
  
  relativeIteration+=1
  if(CURRENT_STATE == State.Wait):
    stateTransition=Wait()
  elif(CURRENT_STATE==State.Forward):
    stateTransition=Forward(car,sensors,relativeIteration,value)
  elif(CURRENT_STATE==State.TurnR):
    stateTransition=TurnR()
  elif(CURRENT_STATE==State.TurnL):
    stateTransition=TurnL()
  elif(CURRENT_STATE==State.RoundAbout):
    stateTransition=RoundAbout()
  elif(CURRENT_STATE==State.Park):
    stateTransition=Park()
  else:
    print("Unknown State")
  
  endTime=time()
  
  if(endTime-startTime<1/120):
    sleep(1/120-(endTime-startTime))
  return

def init():
  global CURRENT_STATE
  sensors.run()
  CURRENT_STATE=State.Forward

def main():
  init()
  while(True):
    iteration()


if __name__=="__main__":
  main()
