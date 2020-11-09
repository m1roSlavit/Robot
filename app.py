import RPi.GPIO as GPIO
import time

GPIO.cleanup()

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

def getAngleCalculatte(degree):
    ms = round(2.5 + ((10 * degree) / 100), 1)
    return ms

try:
  while True:
    deg = int(input("enter degree "))/2
    pwm = GPIO.PWM(17, 50)
    pwm.start(2.5)
    pwm.ChangeDutyCycle(getAngleCalculatte(deg))
    time.sleep(1)
    pwm.stop()
except KeyboardInterrupt:
  pwm.stop()
  GPIO.cleanup()

GPIO.cleanup()