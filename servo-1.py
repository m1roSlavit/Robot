import RPi.GPIO as GPIO
import time

servoPIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization

def get_angle_calculate (degree):
    ms=round(2.5+((10*degree)/180),1)
    return ms
def get_move (degree):
    if degree > 180:
        degree = 180
    elif degree < 0:
        degree = 0  
    ms=get_angle_calculate(degree)
    p.ChangeDutyCycle(ms)
    time.sleep(1)
    print("DONE ")

try:
  while True:
    deg = int(input("enter degree "))
    get_move(deg)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()