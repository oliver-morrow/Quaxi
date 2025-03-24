import cv2
from picamera2 import Picamera2
import threading
from AIRecog import AISensor
from LineTracking import lineTrackingStatus
from ProximitySensor import ProximitySensor
from time import sleep



hasCam=True
def aiThread(obj):
  while(hasCam):
    obj.ai.run(obj.picam2.capture_array())
    obj.ai.ready=True
  obj.ai.ready=True
  return


# each sensor runs in seperate thread
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
    self.aithread.start()
    while(not self.ai.ready):
      sleep(1/120)
      continue
    print("Done setting up sensors")


  def __init__(self,car):
    global hasCam
    self.car=car
    try:
      self.picam2 = Picamera2()
      self.picam2.configure(self.picam2.create_still_configuration(main={"size": (640, 640), "format": "RGB888"}))
      self.picam2.start()
      hasCam=True
    except Exception as e:
      hasCam=False
      self.picam2=None
      print("no camera")
    self.ai=AISensor()

