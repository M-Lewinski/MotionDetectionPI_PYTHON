from picamera.array import PiRGBArray
from picamera import PiCamera
<<<<<<< HEAD
import time
import cv2

=======
import RPi.GPIO as GPIO
import time
import datetime
import cv2

class Movement:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        # Pins
        self.LASER_PIN = 4
        self.STEP_X = 17
        self.DIR_X = 18
        self.STEP_Y = 21
        self.DIR_Y = 22
        self.MOTOR_ENABLE = 25
        self.END_X = 23
        self.END_Y = 24

        # Constant values
        self.STEP = 17.77
        self.DELAY = 10000
        self.MIN_X = -130.0
        self.MIN_Y = -130.0
        self.MAX_X = 130.0
        self.MAX_Y = 130.0

        # Move variables
        self.now_x = 0.0
        self.now_y = 0.0
        self.now_x2 = 0.0
        self.now_y2 = 0.0

        #Pin setup
        GPIO.setup(self.LASER_PIN, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.END_X, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.END_Y, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.STEP_X, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.DIR_X, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.STEP_Y, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.DIR_Y, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.MOTOR_ENABLE, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(self.DIR_X, GPIO.LOW)
        GPIO.output(self.DIR_Y, GPIO.LOW)
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

    def move(self,angle_x,angle_y,scale = 1):
        if angle_x >= 0.0:
            GPIO.output(self.DIR_X, GPIO.HIGH)
        else:
            GPIO.output(self.DIR_X, GPIO.LOW)



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

    def draw_text(self, frame, text, x, y, color=(200, 255, 155), thickness=2, size=1):
        if x is not None and y is not None:
            cv2.putText(
                frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

>>>>>>> cdde3bd9bd332d52d49a280abe1e0b5fa6fcdf89

def cameraControl(config):
    camera = PiCamera()
    res = tuple(config["resolution"])
    camera.resolution = res
    camera.framerate = config["fps"]
    rawCapture = PiRGBArray(camera, size=res)
    time.sleep(config["camera_warmup_time"])
    rememberFrame = None
<<<<<<< HEAD
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        rememberFrame,image = findMotion(image,rememberFrame,config)
=======
    fpsCounter = FPS()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        fps = fpsCounter.fps()
        fpsCounter.draw_text(image, "{:.3f}".format(fps), 30, 30)
        # rememberFrame, image = findMotion(image, rememberFrame, config)
>>>>>>> cdde3bd9bd332d52d49a280abe1e0b5fa6fcdf89
        cv2.imshow("Primary", image)
        rawCapture.truncate(0)  # Clear capture for the next frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
<<<<<<< HEAD
    cv2.destroyAllWindows()


def findMotion(image,rememberFrame,config):
    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grayImage = cv2.GaussianBlur(grayImage, (21, 21), 0)
    if rememberFrame is None:
        return grayImage,image
    frameDelta = cv2.absdiff(rememberFrame, grayImage)
    cv2.imshow("DIFF", frameDelta)

    thresh = cv2.threshold(frameDelta, 10, 255, cv2.THRESH_BINARY)[1]

    thresh = cv2.dilate(thresh, None, iterations=2)
    (_, contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)
=======
    GPIO.cleanup()
    cv2.destroyAllWindows()


def findMotion(image, rememberFrame, config):
    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grayImage = cv2.GaussianBlur(grayImage, (21, 21), 0)
    if rememberFrame is None:
        return grayImage, image
    frameDelta = cv2.absdiff(rememberFrame, grayImage)
    # cv2.imshow("DIFF", frameDelta)

    thresh = cv2.threshold(frameDelta, 35, 255, cv2.THRESH_BINARY)[1]

    thresh = cv2.dilate(thresh, None, iterations=2)
    (_, contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
>>>>>>> cdde3bd9bd332d52d49a280abe1e0b5fa6fcdf89
    figures = []
    for c in contours:
        if cv2.contourArea(c) < config["min_area"]:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        interserct = False
        if not interserct:
<<<<<<< HEAD
            figures.append((x,w,y,h))
=======
            figures.append((x, w, y, h))
>>>>>>> cdde3bd9bd332d52d49a280abe1e0b5fa6fcdf89
    if len(figures) > 0:
        rememberFrame = None
    for fig in figures:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return rememberFrame, image
