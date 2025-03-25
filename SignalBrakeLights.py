from robot_hat import Pin
import time

#Pin Setup:
pin1 = Pin("D0")
pin2 = Pin("D1")

#Function for Signal Lights:
def signal_left(times = 5, interval = 0.5):
    for _ in range(times):
        pin1.value(1)  # set to low level
        time.sleep(interval)
        pin1.value(0)  # set to high level
        time.sleep(interval)

#Function for Signal Lights:
def signal_right(times = 5, interval = 0.5):
    for _ in range(times):
        pin2.value(1)  # set to low level
        time.sleep(interval)
        pin2.value(0)  # set to high level
        time.sleep(interval)
        
if __name__ == "__main__":
    signal_left()
    time.sleep(10)
    signal_right()
