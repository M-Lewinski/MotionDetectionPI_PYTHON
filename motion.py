from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import datetime
import cv2

class Movement:
    def __init__(self):
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
        self.DELA_Y = 500
        self.MIN_X = -66.0
        self.MIN_Y = -66.0
        self.MAX_X = 66.0
        self.MAX_Y = 66.0

        # Move variables
        self.nowX = 0.0
        self.nowY = 0.0
        self.nowX2 = 0.0
        self.nowY2 = 0.0


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


def cameraControl(config):
    camera = PiCamera()
    res = tuple(config["resolution"])
    camera.resolution = res
    camera.framerate = config["fps"]
    rawCapture = PiRGBArray(camera, size=res)
    time.sleep(config["camera_warmup_time"])
    rememberFrame = None
    fpsCounter = FPS()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        fps = fpsCounter.fps()
        fpsCounter.draw_text(image, "{:.3f}".format(fps), 30, 30)
        rememberFrame, image = findMotion(image, rememberFrame, config)
        cv2.imshow("Primary", image)
        rawCapture.truncate(0)  # Clear capture for the next frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
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
    figures = []
    for c in contours:
        if cv2.contourArea(c) < config["min_area"]:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        interserct = False
        if not interserct:
            figures.append((x, w, y, h))
    if len(figures) > 0:
        rememberFrame = None
    for fig in figures:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return rememberFrame, image
