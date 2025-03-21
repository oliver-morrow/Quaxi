import RPi.GPIO as GPIO
import time

# Pin definitions
LEFT_INDICATOR_PIN = 16
RIGHT_INDICATOR_PIN = 17
BRAKE_LIGHT_PIN_1 = 18
BRAKE_LIGHT_PIN_2 = 19
HEADLIGHT_PIN_1 = 20
HEADLIGHT_PIN_2 = 21

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
for pin in [LEFT_INDICATOR_PIN, RIGHT_INDICATOR_PIN, BRAKE_LIGHT_PIN_1, 
           BRAKE_LIGHT_PIN_2, HEADLIGHT_PIN_1, HEADLIGHT_PIN_2]:
    GPIO.setup(pin, GPIO.OUT)

def activate_left_indicator(cycles=5, interval=0.5):
    """Activate left turn indicator light"""
    for _ in range(cycles):
        GPIO.output(LEFT_INDICATOR_PIN, GPIO.HIGH)
        time.sleep(interval)
        GPIO.output(LEFT_INDICATOR_PIN, GPIO.LOW)
        time.sleep(interval)

def activate_right_indicator(cycles=5, interval=0.5):
    """Activate right turn indicator light"""
    for _ in range(cycles):
        GPIO.output(RIGHT_INDICATOR_PIN, GPIO.HIGH)
        time.sleep(interval)
        GPIO.output(RIGHT_INDICATOR_PIN, GPIO.LOW)
        time.sleep(interval)

def set_brake_lights(active=True):
    """Control vehicle brake lights"""
    state = GPIO.HIGH if active else GPIO.LOW
    GPIO.output(BRAKE_LIGHT_PIN_1, state)
    GPIO.output(BRAKE_LIGHT_PIN_2, state)

def set_headlights(active=True):
    """Control vehicle headlights"""
    state = GPIO.HIGH if active else GPIO.LOW
    GPIO.output(HEADLIGHT_PIN_1, state)
    GPIO.output(HEADLIGHT_PIN_2, state)
