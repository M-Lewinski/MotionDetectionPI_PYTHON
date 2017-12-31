import RPi.GPIO as GPIO
import time


class Movement:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        # Pins
        self.LASER_PIN = 38  # CHECKED
        self.STEP_X = 35
        self.DIR_X = 37
        self.STEP_Y = 31
        self.DIR_Y = 33
        self.MOTOR_ENABLE = 29  # CHECKED
        self.END_X = 32
        self.END_Y = 36

        # Constant values
        self.STEP = 17.77
        self.DELAY = 500
        self.MIN_X = -135.0
        self.MIN_Y = -135.0
        self.MAX_X = 120.0
        self.MAX_Y = 120.0

        # Move variables
        self.now_x = 0.0
        self.now_y = 0.0
        self.now_x2 = 0.0
        self.now_y2 = 0.0

        # Pin setup
        GPIO.setup(self.LASER_PIN, GPIO.OUT)
        GPIO.setup(self.END_X, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.END_Y, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.STEP_X, GPIO.OUT)
        GPIO.setup(self.DIR_X, GPIO.OUT)
        GPIO.setup(self.STEP_Y, GPIO.OUT)
        GPIO.setup(self.DIR_Y, GPIO.OUT)
        GPIO.setup(self.MOTOR_ENABLE, GPIO.OUT)
        GPIO.output(self.DIR_X, GPIO.HIGH)
        GPIO.output(self.DIR_Y, GPIO.HIGH)
        GPIO.output(self.MOTOR_ENABLE, GPIO.LOW)

    def laser_on(self):
        GPIO.output(self.LASER_PIN, GPIO.HIGH)

    def laser_off(self):
        GPIO.output(self.LASER_PIN, GPIO.LOW)

    def delay(self, microseconds):
        time.sleep(microseconds / 1000000)

    def calibrate(self):
        self.laser_off()
        while GPIO.input(self.END_X) == GPIO.HIGH:
            GPIO.output(self.STEP_X, GPIO.HIGH)
            self.delay(self.DELAY)
            GPIO.output(self.STEP_X, GPIO.LOW)
            self.delay(self.DELAY)
        while GPIO.input(self.END_Y) == GPIO.HIGH:
            GPIO.output(self.STEP_Y, GPIO.HIGH)
            self.delay(self.DELAY)
            GPIO.output(self.STEP_Y, GPIO.LOW)
            self.delay(self.DELAY)
        self.now_x = self.MIN_X
        self.now_y = self.MIN_Y
        self.move(-self.now_x, -self.now_y, calibration=True)

    def move(self, angle_x, angle_y, multiplier=1, calibration=False):
        positive_x = True
        positive_y = True
        if angle_x >= 0.0:
            GPIO.output(self.DIR_X, GPIO.LOW)
        else:
            GPIO.output(self.DIR_X, GPIO.HIGH)
            positive_x = False
        if angle_y >= 0.0:
            GPIO.output(self.DIR_Y, GPIO.LOW)
        else:
            GPIO.output(self.DIR_Y, GPIO.HIGH)
            positive_y = False
        angle_x = abs(angle_x)
        angle_y = abs(angle_y)

        while calibration or GPIO.input(self.END_X) == GPIO.HIGH or GPIO.input(self.END_Y) == GPIO.HIGH:
            if angle_x > 0.0:
                GPIO.output(self.STEP_X, GPIO.HIGH)
                if positive_x is True:
                    self.now_x += 1.0 / self.STEP
                else:
                    self.now_x -= 1.0 / self.STEP
                angle_x -= 1.0 / self.STEP
            if angle_y > 0.0:
                GPIO.output(self.STEP_Y, GPIO.HIGH)
                if positive_y is True:
                    self.now_y += 1.0 / self.STEP
                else:
                    self.now_y -= 1.0 / self.STEP
                angle_y -= 1.0 / self.STEP
            self.delay(self.DELAY / multiplier)
            GPIO.output(self.STEP_X, GPIO.LOW)
            GPIO.output(self.STEP_Y, GPIO.LOW)
            self.delay(self.DELAY / multiplier)
            if not (self.MIN_X <= self.now_x <= self.MAX_X) or not (self.MIN_Y <= self.now_y <= self.MAX_Y):
                return False
            if angle_x <= 0.0 and angle_y <= 0.0:
                break
        return True

    def center(self):
        self.move(-self.now_x, -self.now_y, multiplier=4)
        time.sleep(1)
