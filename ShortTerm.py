from time import time,sleep
from Sensors import Sensors
from DrivingModule import CarController
from picarx import Picarx
from globals import Zone,State,Node,CURRENT_STATE,CURRENT_ZONE
from SignalBrakeLights import signal_left, brake_light, main_light
from robot_hat import Music
from Forward import Forward

# Initialize sound system
music = Music()
music.music_set_volume(20)

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
  brake_light(True)  # Activate brake lights while waiting
  return True
  
def TurnR():
  # Signal right turn with lights
  brake_light(False)
  music.sound_play_threading('../sounds/turn.wav')
  car.turn_right()
  return 1
  
def TurnL():
  # Signal left turn with lights
  brake_light(False)
  signal_left(times=2, interval=0.3)
  music.sound_play_threading('../sounds/turn.wav')
  car.turn_left()
  return 1
  
def RoundAbout():
  # Signal for roundabout
  brake_light(True)
  sleep(0.5)
  brake_light(False)
  signal_left(times=3, interval=0.2)
  car.turn_right(angle=30, speed=20)
  return 1
  
def Park():
  # Parking procedure
  brake_light(True)
  music.sound_play_threading('../sounds/brake.wav')
  car.stop()
  sleep(1)
  return 1
 
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
  # Position camera to face forward at ideal angle
  print("Adjusting camera position...")
  picarx.set_cam_pan_angle(0)  # Center horizontally
  picarx.set_cam_tilt_angle(-15)  # Tilt down slightly to see the road
  
  sensors.run()
  main_light(True)  # Turn on main lights when starting
  music.sound_play_threading('../sounds/startup.wav')
  CURRENT_STATE=State.Forward

def main():
  init()
  while(True):
    iteration()


if __name__=="__main__":
  main()
