import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

servo_pins = [[17, 27], [22, 23], [24, 25], [5, 6]]  # [[joint_motor1, leg_motor1], [joint_motor2, leg_motor2]...]

GPIO.setup(servo_pins[0][0], GPIO.OUT)
GPIO.setup(servo_pins[0][1], GPIO.OUT)
GPIO.setup(servo_pins[1][0], GPIO.OUT)
GPIO.setup(servo_pins[1][1], GPIO.OUT)
GPIO.setup(servo_pins[2][0], GPIO.OUT)
GPIO.setup(servo_pins[2][1], GPIO.OUT)
GPIO.setup(servo_pins[3][0], GPIO.OUT)
GPIO.setup(servo_pins[3][1], GPIO.OUT)

motor1_joint = GPIO.PWM(servo_pins[0][0], 50)
motor1_leg = GPIO.PWM(servo_pins[0][1], 50)
motor2_joint = GPIO.PWM(servo_pins[1][0], 50)
motor2_leg = GPIO.PWM(servo_pins[1][1], 50)
motor3_joint = GPIO.PWM(servo_pins[2][0], 50)
motor3_leg = GPIO.PWM(servo_pins[2][1], 50)
motor4_joint = GPIO.PWM(servo_pins[3][0], 50)
motor4_leg = GPIO.PWM(servo_pins[3][1], 50)

motor1_joint.start(6)
motor1_leg.start(2.5)
motor2_joint.start(6)
motor2_leg.start(2.5)
motor3_joint.start(6)
motor3_leg.start(2.5)
motor4_joint.start(6)
motor4_leg.start(2.5)


def getAngleCalculatte(degree):
    ms = round(6 + ((10 * degree) / 100), 1)
    return ms


# pwm.ChangeDutyCycle(getAngleCalculatte(30))

try:
    while True:
        deg1 = int(input("enter degree motor1_joint "))
        motor1_joint.ChangeDutyCycle(getAngleCalculatte(deg1))
        deg2 = int(input("enter degree motor1_leg"))
        motor1_leg.ChangeDutyCycle(getAngleCalculatte(deg2))
        deg3 = int(input("enter degree motor2_joint"))
        motor2_joint.ChangeDutyCycle(getAngleCalculatte(deg3))
        deg4 = int(input("enter degree motor2_leg"))
        motor2_leg.ChangeDutyCycle(getAngleCalculatte(deg4))
        deg5 = int(input("enter degree motor3_joint"))
        motor3_joint.ChangeDutyCycle(getAngleCalculatte(deg5))
        deg6 = int(input("enter degree motor3_leg"))
        motor3_leg.ChangeDutyCycle(getAngleCalculatte(deg6))
        deg7 = int(input("enter degree motor4_joint"))
        motor4_joint.ChangeDutyCycle(getAngleCalculatte(deg7))
        deg8 = int(input("enter degree motor4_leg"))
        motor4_leg.ChangeDutyCycle(getAngleCalculatte(deg8))

except KeyboardInterrupt:
    motor1_joint.stop()
    motor1_leg.stop()
    motor2_joint.stop()
    motor2_leg.stop()
    motor3_joint.stop()
    motor3_leg.stop()
    motor4_joint.stop()
    motor4_leg.stop()
    GPIO.cleanup()
