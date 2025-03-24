import RPi.GPIO as GPIO #Python 3 Library for controlling I/O pins on RPI 4
import time

#Pin Setup:
Left_Signal = 16
Right_Signal = 17
Brake_Light1 = 18
Brake_Light2 = 19
Main_Light1 = 20
Main_Light2 = 21


#Setup pins to be output values (Brake | Signals | Main):
GPIO.setmode(GPIO.BCM)
GPIO.setup(Left_Signal, GPIO.OUT)
GPIO.setup(Right_Signal, GPIO.OUT)
GPIO.setup(Brake_Light1, GPIO.OUT)
GPIO.setup(Brake_Light2, GPIO.OUT)
GPIO.setup(Main_Light1, GPIO.OUT)
GPIO.setup(Main_Light2, GPIO.OUT)

#Function for Signal Lights:
def signal_left(times = 5, interval = 0.5):
    for _ in range(times):
        GPIO.output(Left_Signal, GPIO.HIGH)
        time.sleep(interval)
        GPIO.output(Left_Signal, GPIO.LOW)
        time.sleep(interval)

#Function for Break Lights:
def brake_light(state=True):
    GPIO.output(Brake_Light1, GPIO.HIGH if state else GPIO.LOW)
    GPIO.output(Brake_Light2, GPIO.HIGH if state else GPIO.LOW)

#Function for Main Lights:
def main_light(state=True):
    GPIO.output(Main_Light1, GPIO.HIGH if state else GPIO.LOW)
    GPIO.output(Main_Light2, GPIO.HIGH if state else GPIO.LOW)
