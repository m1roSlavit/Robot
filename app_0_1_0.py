import RPi.GPIO as GPIO
import time

SERVO_FREQUENCY = 50
START_POSITION = 2.5

GPIO.cleanup()

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

def getAngleCalculatte(angle):
  ms = round(2.5 + ((10 * angle) / 100), 1)
  return ms

def setServoAngle(pin_number, angle, start_position = START_POSITION):
  pwm = GPIO.PWM(pin_number, SERVO_FREQUENCY)
  pwm.start(start_position)
  pwm.ChangeDutyCycle(getAngleCalculatte(angle))
  time.sleep(0.3)
  pwm.stop()
  
try:
  while True:
    ang = int(input("enter angle "))/2
    setServoAngle(17, ang)
except KeyboardInterrupt:
  GPIO.cleanup()

GPIO.cleanup()