import RPi.GPIO as GPIO
import time
import threading

print("\n░░░░░▄▀▀▀▄░░░░░░░░\n▄███▀░◐░░░▌░░░░░░\n░░░░▌░░░░░▐░░░░░░░\n░░░░▐░░░░░▐░░░░░░░\n░░░░▌░░░░░▐▄▄░░░░░\n░░░░▌░░░░▄▀▒▒▀▀▀▀▄\n░░░▐░░░░▐▒▒▒▒▒▒▒▒▀▀▄\n░░░▐░░░░▐▄▒▒▒▒▒▒▒▒▒▒▀▄\n░░░░▀▄░░░░▀▄▒▒▒▒▒▒▒▒▒▒▀▄\n░░░░░░▀▄▄▄▄▄█▄▄▄▄▄▄▄▄▄▄▄▀▄\n░░░░░░░░░░░▌▌░▌▌░░░░░\n░░░░░░░░░░░▌▌░▌▌░░░░░\n░░░░░░░░░▄▄▌▌▄▌▌░░░░░""")

servo_pins = [[17, 27], [22, 23], [24, 25], [5, 19]]
servo_reversed = [[True, False], [False, True], [True, False], [False, True]]

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ports = [[0,0],[0,0],[0,0],[0,0]]


class Motor:
    def __init__(self, pin_number, is_reverse=False, servo_frequency=50, start_position=2.5):
        self.pin_number = pin_number
        self.servo_frequency = servo_frequency
        self.start_position = start_position
        self.is_reverse = is_reverse
        GPIO.setup(pin_number, GPIO.OUT)

    @staticmethod
    def get_angle_calculate(angle):
        ms = round(2.5 + ((10 * angle) / 180), 1)
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

    def move_motor(self, angle, name=None):
        right_angle = (180 - angle) if self.is_reverse else angle
        self.set_servo_angle(right_angle)


class LegsFacade:
    def __init__(self, pins, servos_reversed, servo_frequency, start_position):
        self.leg1hip = Motor(pins[0][0], servos_reversed[0][0], servo_frequency, start_position)
        self.leg1knee = Motor(pins[0][1], servos_reversed[0][1], servo_frequency, start_position)
        self.leg2hip = Motor(pins[1][0], servos_reversed[1][0], servo_frequency, start_position)
        self.leg2knee = Motor(pins[1][1], servos_reversed[1][1], servo_frequency, start_position)
        self.leg3hip = Motor(pins[2][0], servos_reversed[2][0], servo_frequency, start_position)
        self.leg3knee = Motor(pins[2][1], servos_reversed[2][1], servo_frequency, start_position)
        self.leg4hip = Motor(pins[3][0], servos_reversed[3][0], servo_frequency, start_position)
        self.leg4knee = Motor(pins[3][1], servos_reversed[3][1], servo_frequency, start_position)


class Robot:
    def __init__(self, pins, servos_reversed, servo_frequency=50, start_position=2.5):
        self.legs = LegsFacade(pins, servos_reversed, servo_frequency, start_position)

    def set_start_position(self):
        self.legs.leg1hip.move_motor(90)
        self.legs.leg1knee.move_motor(10)
        self.legs.leg2hip.move_motor(90)
        self.legs.leg2knee.move_motor(10)
        self.legs.leg3hip.move_motor(90)
        self.legs.leg3knee.move_motor(10)
        self.legs.leg4hip.move_motor(90)
        self.legs.leg4knee.move_motor(10)

    def test(self):
        thread_list = []
        t1 = threading.Thread(target=self.legs.leg1hip.move_motor, name='1', args=(60, '1'))
        thread_list.append(t1)
        t2 = threading.Thread(target=self.legs.leg2hip.move_motor, name='2', args=(60, '2'))
        thread_list.append(t2)
        t1.start()
        t2.start()
        for t in thread_list:
            t.join()


try:
    myRobot = Robot(servo_pins, servos_reversed=servo_reversed)
except KeyboardInterrupt:
    pass
    GPIO.cleanup()

GPIO.cleanup()
