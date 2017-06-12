from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import math
import datetime
import cv2
import copy

class Movement:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        # Pins
        self.LASER_PIN = 38 #CHECKED
        self.STEP_X = 35
        self.DIR_X = 37
        self.STEP_Y = 31
        self.DIR_Y = 33
        self.MOTOR_ENABLE = 29 #CHECKED
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

        #Pin setup
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

    def delay(self,microseconds):
        time.sleep(microseconds/1000000)

    def calibrate(self):
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

    def move(self,angle_x,angle_y,multiplier=1, calibration=False):
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
                    self.now_x += 1.0/self.STEP
                else:
                    self.now_x -= 1.0/self.STEP
                angle_x -= 1.0/self.STEP
            if angle_y > 0.0:
                GPIO.output(self.STEP_Y, GPIO.HIGH)
                if positive_y is True:
                    self.now_y += 1.0 / self.STEP
                else:
                    self.now_y -= 1.0 / self.STEP
                angle_y -= 1.0 / self.STEP
            self.delay(self.DELAY/multiplier)
            GPIO.output(self.STEP_X, GPIO.LOW)
            GPIO.output(self.STEP_Y, GPIO.LOW)
            self.delay(self.DELAY/multiplier)
            if not (self.MIN_X <= self.now_x <= self.MAX_X) or not (self.MIN_Y <= self.now_y <= self.MAX_Y):
                return False
            if angle_x <= 0.0 and angle_y <= 0.0:
                break
        return True

    def center(self):
        self.move(-self.now_x, -self.now_y, multiplier=4)


class FPS:
    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0
        self.start()

    def start(self):
        # start the timer
        if self._start is None:
            self._start = datetime.datetime.now()
        return self

    def end(self):
        # check end time
        self._end = datetime.datetime.now()

    def update(self):
        # increment the total number of frames examined during the start and end intervals
        self._numFrames += 1

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start).total_seconds()

    def fps(self):
        self.update()
        self.end()
        seconds = self.elapsed()
        fps = self._numFrames / self.elapsed()
        if seconds > 1:
            self._start = self._end
            self._numFrames = 0
        # compute the (approximate) frames per second
        return fps

    def draw_text(self, frame, text, x, y, color=(120, 255, 0), thickness=2, size=1):
        if x is not None and y is not None:
            cv2.putText(
                frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)


def cameraControl(config):
    movement = Movement()
    movement.calibrate()
    camera = PiCamera()
    res = tuple(config["resolution"])
    camera.resolution = res
    camera.framerate = config["fps"]
    rawCapture = PiRGBArray(camera, size=res)
    time.sleep(config["camera_warmup_time"])
    rememberFrame = None
    fpsCounter = FPS()
    startTime = None
    endTime = None
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        # fps = fpsCounter.fps()
        # fpsCounter.draw_text(image, "{:.3f}".format(fps), 30, 30)
        
        if startTime is None:
            startTime = datetime.datetime.now()
        else:
            endTime = datetime.datetime.now()
        
        rememberFrame = findMotion(image, rememberFrame, config, movement)
        if rememberFrame is None:
            startTime = None
            endTime = None
        if endTime is not None:
            if (endTime - startTime).total_seconds() >= 5.0:
                movement.center()
                startTime = None
                endTime = None
        if config['show_video'] is True:
            cv2.imshow("Primary", image)
        rawCapture.truncate(0)  # Clear capture for the next frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    GPIO.cleanup()
    cv2.destroyAllWindows()


def findMotion(image, rememberFrame, config, movement):
    image_cpy = copy.copy(image)

    grayImage = cv2.cvtColor(image_cpy, cv2.COLOR_BGR2GRAY)
    grayImage = cv2.GaussianBlur(grayImage, (21, 21), 0)
    if rememberFrame is None:
        # return grayImage.copy().astype("float"), image
        return grayImage
    #cv2.accumulateWeighted(grayImage, rememberFrame, 0.5)
    #frameDelta = cv2.absdiff(grayImage, cv2.convertScaleAbs(rememberFrame))
    frameDelta = cv2.absdiff(grayImage, rememberFrame)

    cv2.imshow("DIFF", frameDelta)

    thresh = cv2.threshold(frameDelta, 15, 255, cv2.THRESH_BINARY)[1]

    thresh = cv2.dilate(thresh, None, iterations=2)
    (_, contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
    figures = []
    for c in contours:
        if cv2.contourArea(c) < config["min_area"]:
            continue
        pos_x, pos_y, width, height = cv2.boundingRect(c)
        interserct = False
        if not interserct:
            figures.append({'Pos_x': pos_x, 'Pos_y': pos_y, 'width': width, 'height': height})
    #if len(figures) > 0:
    #    rememberFrame = None
    
    target = None
    for fig in figures:
        fig['delta_x'] = (fig['Pos_x'] + (fig['width']/2)) - (config["resolution"][0]/2)
        fig['delta_y'] = (fig['Pos_y'] + (fig['height']/2)) - (config["resolution"][1]/2)
        fig['distance'] = math.sqrt(fig['delta_x']**2 + fig['delta_y']**2)
        if target is None:
            target = fig
        else:
            #if target['distance'] < fig['distance']:
            if abs(target["delta_x"]*target["delta_y"]) < abs(fig["delta_x"]*fig["delta_y"]):
                target = fig
        cv2.rectangle(image, (fig['Pos_x'], fig['width']), (fig['Pos_x'] + fig['Pos_y'], fig['width']+ fig['height']), (0, 255, 0), 2)
    if target is not None:
        print(target)
        if abs(target['delta_x']) > config['delta'] or abs(target['delta_y']) > config['delta']:
            # if abs(target['delta_x']) < config['delta']: target['delta_x'] = 0.0
            # if abs(target['delta_y']) < config['delta']: target['delta_y'] = 0.0
            angle_x = (target['delta_x']/config["resolution"][0])*config['angle_view'][0]
            angle_y = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]

            inBoundries = movement.move(angle_x,angle_y,multiplier=10)
            if inBoundries is False:
                movement.center()
            movement.laser_on()
            movement.delay(5000)
            movement.laser_off()
            movement.delay(10000)
            return None

    return grayImage
