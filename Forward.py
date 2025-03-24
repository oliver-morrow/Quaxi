import re
import math
from globals import State,NEXT_STATE,NODE_COORDS


# V1 classes
classes=['duck_regular', 'sign_noentry', 'sign_oneway_left', 'sign_oneway_right', 'sign_stop', 'sign_yield']
# full classes
# classes=['duck_regular','duck_specialty','sign_stop','sign_oneway_right','sign_oneway_left','sign_noentry','sign_yield','road_crosswalk','road_oneway','vehicle']


MAX_ANGLE=40
camAngle=0
DrivingSpeed=30
angle=0
TIME_TO_TURN=20
turnTimer=0
timeToLive=0

# move camera and return closest sign
def trackSign(aiData, controller):
    global camAngle
    if not aiData:
        return None

    largestArea = 0
    closestSign = None
    delta = 0

    for obj in aiData:
        # Skip objects with low confidence
        if obj.score < 0.7:
            continue
            
        # Check if it's a sign
        if "sign_" in obj.label:
            # Calculate area of bounding box
            width = obj.bbox.xmax - obj.bbox.xmin
            height = obj.bbox.ymax - obj.bbox.ymin
            area = width * height
            
            if area > largestArea:
                largestArea = area
                closestSign = obj
                
        # Calculate camera angle adjustment
        center_x = (obj.bbox.xmin + obj.bbox.xmax) / 2
        normalized_x = center_x / 640  # Assuming 640px width image
        
        if normalized_x > 0.5:
            delta = controller.speed/15 - math.exp(-camAngle/MAX_ANGLE/2)
        else:
            delta = -controller.speed/15 - math.exp(camAngle/MAX_ANGLE/2)
        delta *= 40/180
        print(delta)

    camAngle = max(min(MAX_ANGLE, camAngle+delta), -MAX_ANGLE)
    print(camAngle)
    if closestSign is None:
        camAngle = 0

    controller.turnCam(camAngle)
    return closestSign

def dodge():
  return False  # This needs implementation for obstacle avoidance

# return true if unable to continue without checking long term, false otherwise
def Forward(controller,sensors,iteration,distance):
  global angle
  global camAngle

  global drivingSpeed
  global turnTimer
  global timeToLive
  hardware=sensors.ReadHardware()
  
  #print(hardware["lineTracker"])
  if (turnTimer<=0):
    angle=0
  if (hardware["lineTracker"][0]==1 and
    hardware["lineTracker"][2]==1):
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][0]==1 and turnTimer<=0):
    angle=MAX_ANGLE
    turnTimer=TIME_TO_TURN
  elif(hardware["lineTracker"][2]==1 and turnTimer<=0):
    angle=-1*MAX_ANGLE
    turnTimer=TIME_TO_TURN
  turnTimer=max(turnTimer-1,0)
  
  # check image tracking for lines ig
  aiData=sensors.ReadAI()

  sign=trackSign(aiData,controller)

  # store last known sign somewhere since cam might see it when we are besides it
  if sign is not None:
    signType = sign.label
    if signType == 'sign_noentry':
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
  if(dodge()):
    return 0
  
  controller.turn_right(angle=angle,speed=DrivingSpeed)
 
  if(timeToLive==0):
    # Adjust the travel time factor based on your car's speed.
    travel_time = distance / 20.0  # This factor may need tuning.
    # approx 120 iteration per second
    timeToLive = travel_time*120
  
  if(iteration>=timeToLive):
    timeToLive=0
    return 1

  return 0
