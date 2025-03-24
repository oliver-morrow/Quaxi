import cv2
from picamera2 import Picamera2
import threading
from AIRecog import AISensor
from LineTracking import lineTrackingStatus
from ProximitySensor import ProximitySensor
from time import sleep
from aiymakerkit import vision

hasCam = True

def aiThread(obj):
    while(hasCam):
        # Get frame directly from aiymakerkit
        try:
            for frame in vision.get_frames():
                obj.ai.run(frame)
                # Optional: draw bounding boxes for debugging
                if obj.ai.results and obj.ai.labels:
                    vision.draw_objects(frame, obj.ai.results, obj.ai.labels)
                break  # Process only one frame at a time
        except Exception as e:
            print(f"Error getting camera frame: {e}")
            sleep(0.1)
    obj.ai.ready = True
    return


# each sensor runs in seperate thread
class Sensors:
    def ReadAI(self):
        return self.ai.results
    
    def ReadHardware(self):
        result = {"proximity": ProximitySensor(self.car), "lineTracker": lineTrackingStatus(self.car)}
        return result
    
    def ReadImgProcessing(self):
        return 0


    def run(self):
        self.aithread = threading.Thread(target=aiThread, args=(self,))
        self.aithread.start()
        while(not self.ai.ready):
            sleep(1/120)
            continue
        print("Done setting up sensors")


    def __init__(self, car):
        global hasCam
        self.car = car
        try:
            # We don't need picamera2 anymore as aiymakerkit handles camera access
            hasCam = True
        except Exception as e:
            hasCam = False
            print(f"No camera available: {e}")
        self.ai = AISensor()

