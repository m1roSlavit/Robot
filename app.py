import RPi.GPIO as GPIO
import time

servo_pins = [[17, 27], [22, 23], [24, 25], [5, 6]]

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ports = [[0,0],[0,0],[0,0],[0,0]]


class Motor:
    def __init__(self, pin_number, servo_frequency=50, start_position=2.5):
        self.pin_number = pin_number
        self.servo_frequency = servo_frequency
        self.start_position = start_position
        GPIO.setup(pin_number, GPIO.OUT)

    @staticmethod
    def get_angle_calculate(angle):
        ms = round(2.5 + ((10 * angle) / 100), 1)
        return ms

    def set_servo_angle(self, angle):
        try:
            pwm = GPIO.PWM(self.pin_number, self.servo_frequency)
            pwm.start(self.start_position)
            pwm.ChangeDutyCycle(self.get_angle_calculate(angle))
            time.sleep(0.3)
            pwm.stop()
        except KeyboardInterrupt:
            pwm.stop()
            GPIO.cleanup()


class Leg:
    def __init__(self, pins, servo_frequency, start_position):
        self.hip = Motor(pins[0], servo_frequency, start_position)
        self.knee = Motor(pins[1], servo_frequency, start_position)


class LegsFacade:
    def __init__(self, pins, servo_frequency, start_position):
        self.leg1 = Leg(pins[0], servo_frequency, start_position)
        self.leg2 = Leg(pins[1], servo_frequency, start_position)
        self.leg3 = Leg(pins[2], servo_frequency, start_position)
        self.leg4 = Leg(pins[3], servo_frequency, start_position)


class Robot:
    def __init__(self, pins, servo_frequency=50, start_position=2.5):
        self.all_legs = LegsFacade(pins, servo_frequency, start_position)


try:
    myRobot = Robot(servo_pins)
    while True:
        pass
except KeyboardInterrupt:
    GPIO.cleanup()

GPIO.cleanup()