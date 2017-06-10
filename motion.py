from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2


def cameraControl(config):
    camera = PiCamera()
    res = tuple(config["resolution"])
    camera.resolution = res
    camera.framerate = config["fps"]
    rawCapture = PiRGBArray(camera, size=res)
    time.sleep(config["camera_warmup_time"])
    rememberFrame = None
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        rememberFrame,image = findMotion(image,rememberFrame,config)
        cv2.imshow("Primary", image)
        rawCapture.truncate(0)  # Clear capture for the next frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
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
    figures = []
    for c in contours:
        if cv2.contourArea(c) < config["min_area"]:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        interserct = False
        if not interserct:
            figures.append((x,w,y,h))
    if len(figures) > 0:
        rememberFrame = None
    for fig in figures:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return rememberFrame, image
