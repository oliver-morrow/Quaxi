import re
import math
from globals import State,NEXT_STATE,NODE_COORDS


# V1 classes
classes=['duck_regular', 'sign_noentry', 'sign_oneway_left', 'sign_oneway_right', 'sign_stop', 'sign_yield']
# full classes
# classes=['duck_regular','duck_specialty','sign_stop','sign_oneway_right','sign_oneway_left','sign_noentry','sign_yield','road_crosswalk','road_oneway','vehicle']


MAX_ANGLE=40
ANGULAR_SPEED = 60
camAngle=0
DrivingSpeed=15
angle=0
TIME_TO_TURN=11
turnTimer=0
timeToLive=0

PAUSE_TIME=20
pauseTimer=0
wasStopped=False

# move camera and return closest sign
def trackSign(aiData, controller):
    global camAngle
    if(aiData is None):
        return None

    largestArea = 0
    closestSign = None
    delta = 0

    for r in aiData:
        for b in r.boxes:
            if(b.conf < 0.40):  # Lower threshold for AIYMakerKit
                continue
            if(float(b.xywh[0][2]) * float(b.xywh[0][3]) > largestArea):
                if(re.search("^sign_.*$", classes[int(b.cls[0])]) is not None):
                    largestArea = b.xywh[0][2] * b.xywh[0][3]
                    closestSign = b
            if(float(b.xywhn[0][0]) > 0.5):
                delta = controller.speed/15 - math.exp(-camAngle/MAX_ANGLE/2)
            else:
                delta = -controller.speed/15 - math.exp(camAngle/MAX_ANGLE/2)
            delta *= 40/180

    camAngle = max(min(MAX_ANGLE, camAngle+delta), -MAX_ANGLE)
    if(closestSign is None):
        camAngle = 0

    controller.turnCam(camAngle)
    return closestSign

def dodge(hardware):
  if (hardware["proximity"]>0 and hardware["proximity"]<25):
    return True
    
  return False

def TurnR(controller, sensors, iteration, angular_distance):
  print("wow we going right")
  global DrivingSpeed
  global turnTimer # turning as a result of greyscale line sensor
  global timeToLive
  global pauseTimer
  global wasStopped

  hardware=sensors.ReadHardware()
 
  if(timeToLive==0):
    # Adjust the travel time factor based on your car's speed.
    travel_time = angular_distance / ANGULAR_SPEED
    # approx 120 iteration per second
    timeToLive = travel_time*120
    turnTimer = 0

  if (turnTimer<=0):
    angle = MAX_ANGLE

  if (hardware["lineTracker"][0]==1 and
      hardware["lineTracker"][1]==1 and
      hardware["lineTracker"][2]==1 and
      wasStopped==False):
    pauseTimer=PAUSE_TIME
    wasStopped=True
    return 0
  elif (hardware["lineTracker"][0]==1 and
    hardware["lineTracker"][2]==1):
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][0]==1 and turnTimer<=0):
    angle=60
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][2]==1 and turnTimer<=0):
    angle=-1*60 + MAX_ANGLE
    turnTimer=TIME_TO_TURN
  turnTimer=max(turnTimer-1,0)

  if(wasStopped==True and pauseTimer<-10):
    wasStopped=False

  pauseTimer-=1

  controller.turn_right(angle=angle,speed=0 if pauseTimer>0 else DrivingSpeed)

  if(iteration>=timeToLive):
    timeToLive = 0
    return 1
  return 0

def TurnL(controller, sensors, iteration, angular_distance):
  print("wow we going right")
  global DrivingSpeed
  global turnTimer # turning as a result of greyscale line sensor
  global timeToLive
  global pauseTimer
  global wasStopped

  hardware=sensors.ReadHardware()
 
  if(timeToLive==0):
    # Adjust the travel time factor based on your car's speed.
    travel_time = angular_distance / ANGULAR_SPEED
    # approx 120 iteration per second
    timeToLive = travel_time*120
    turnTimer = 0

  if (turnTimer<=0):
    angle = MAX_ANGLE

  if (hardware["lineTracker"][0]==1 and
      hardware["lineTracker"][1]==1 and
      hardware["lineTracker"][2]==1 and
      wasStopped==False):
    pauseTimer=PAUSE_TIME
    wasStopped=True
    return 0
  elif (hardware["lineTracker"][0]==1 and
    hardware["lineTracker"][2]==1):
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][0]==1 and turnTimer<=0):
    angle=-1*60 + MAX_ANGLE
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][2]==1 and turnTimer<=0):
    angle=60
    turnTimer=TIME_TO_TURN
  turnTimer=max(turnTimer-1,0)

  if(wasStopped==True and pauseTimer<-10):
    wasStopped=False

  pauseTimer-=1

  controller.turn_left(angle=angle,speed=0 if pauseTimer>0 else DrivingSpeed)

  if(iteration>=timeToLive):
    timeToLive = 0
    return 1
  return 0

# return true if unable to continue without checking long term, false otherwise
def Forward(controller,sensors,iteration,distance):
  global angle
  global camAngle

  global DrivingSpeed
  global turnTimer
  global timeToLive
  global pauseTimer
  global wasStopped
  
  hardware=sensors.ReadHardware()

  if(timeToLive==0):
    # Adjust the travel time factor based on your car's speed.
    travel_time = distance / 50.0  # This factor may need tuning.
    # approx 120 iteration per second
    timeToLive = travel_time*120
    camAngle=0
    angle=0
    turnTimer=0

  if (turnTimer<=0):
    angle=-5
  
  if (hardware["lineTracker"][0]==1 and
      hardware["lineTracker"][1]==1 and
      hardware["lineTracker"][2]==1 and
      wasStopped==False):
    pauseTimer=PAUSE_TIME
    wasStopped=True
    return 0
  elif (hardware["lineTracker"][0]==1 and
    hardware["lineTracker"][2]==1):
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][0]==1 and turnTimer<=0):
    angle=MAX_ANGLE
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][2]==1 and turnTimer<=0):
    angle=-1*MAX_ANGLE
    turnTimer=TIME_TO_TURN
  turnTimer=max(turnTimer-1,0)
  
  if(wasStopped==True and pauseTimer<-10):
    wasStopped=False

  # check image tracking for lines ig
  aiData=sensors.ReadAI()

  sign=trackSign(aiData,controller)

  # store last known sign somewhere since cam might see it when we are besides it
  if(sign is not None and 0):# at sign
    signType=classes[int(sign.cls[0])]
    if signType=='sign_noentry':
      return 1
    elif signType == 'sign_oneway_left':
      if(NEXT_STATE==State.TurnR):
        print("wrong way one way")
      return 1
    elif signType == 'sign_oneway_right':
      if(NEXT_STATE==State.TurnL):
        print("wrong way one way")
      return 1
    elif signType == 'sign_stop':
      return 0
    elif signType == 'sign_yield':
      return 0

  # avoid things on read
  pauseTimer-=1
  if(dodge(hardware)):
    controller.turn_right(0,0)
    return 0

  if(angle!=0):
    DrivingSpeed=5
  else:
    DrivingSpeed=15
  controller.turn_right(angle=angle,speed=0 if pauseTimer>0 else DrivingSpeed)
 
  if(iteration>=timeToLive):
    timeToLive=0
    return 1

  return 0
