import threading
from AIRecog import AISensor
from LineTracking import lineTrackingStatus
from ProximitySensor import ProximitySensor
from time import sleep

def aiThread(obj):
  while True:
    if obj.ai.ready:
      # AIY Maker Kit handles the camera frames automatically
      # Just sleep to avoid CPU overload
      sleep(1/30)  # 30fps max refresh rate
    else:
      obj.ai.ready = True
      sleep(1/120)
  return

# each sensor runs in separate thread
class Sensors:
  def ReadAI(self):
    return self.ai.results
    
  def ReadHardware(self):
    result={"proximity":ProximitySensor(self.car),"lineTracker":lineTrackingStatus(self.car)}
    return result
    
  def ReadImgProcessing(self):
    return 0

  def run(self):
    self.aithread=threading.Thread(target=aiThread,args=(self,))
    self.aithread.daemon = True  # Make thread exit when main program exits
    self.aithread.start()
    while(not self.ai.ready):
      sleep(1/120)
      continue
    print("Done setting up sensors")

  def __init__(self,car):
    self.car=car
    # Initialize AI sensor (using AIY Maker Kit)
    try:
      self.ai=AISensor()
      self.ai.ready = False
      print("AI sensor initialized")
    except Exception as e:
      print(f"Error initializing AI sensor: {e}")
      # Create a dummy AI sensor if initialization fails
      self.ai = AISensor()
      self.ai.ready = True

