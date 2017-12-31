import datetime
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import cv2
import copy
import math

from utils.fps import FPS, draw_text
from utils.motion import Movement
import time


def camera_control(config, debug=False):
    movement = Movement()
    movement.calibrate()

    camera = PiCamera()
    camera.resolution = tuple(config["resolution"])
    camera.framerate = config["fps"]

    raw_capture = PiRGBArray(camera, size=camera.resolution)
    time.sleep(config["camera_warmup_time"])
    remember_frame = None
    fps_counter = FPS()
    start_time, end_time = datetime.datetime.now(), datetime.datetime.now()
    try:
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            image = frame.array
            end_time = datetime.datetime.now()

            remember_frame = find_motion(image, remember_frame, config, movement, debug)
            if remember_frame is None:
                start_time = datetime.datetime.now()
                end_time = None

            if end_time is not None:
                if (end_time - start_time).total_seconds() >= 5.0:
                    movement.center()
                    start_time = datetime.datetime.now()
                    remember_frame = None

            if debug:
                fps = fps_counter.fps()
                draw_text(image, "{:.3f}".format(fps), 30, 30)
                cv2.imshow("Primary", image)

            raw_capture.truncate(0)  # Clear capture for the next frame
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        GPIO.cleanup()
        cv2.destroyAllWindows()


def find_motion(image, remember_frame, config, movement, debug=False):
    image_cpy = copy.copy(image)

    gray_image = cv2.cvtColor(image_cpy, cv2.COLOR_BGR2GRAY)
    gray_image = cv2.GaussianBlur(gray_image, (21, 21), 0)
    if remember_frame is None:
        # return gray_image.copy().astype("float"), image
        return gray_image
    # cv2.accumulateWeighted(gray_image, rememberFrame, 0.5)
    # frame_delta = cv2.absdiff(gray_image, cv2.convertScaleAbs(rememberFrame))
    frame_delta = cv2.absdiff(gray_image, remember_frame)

    if debug:
        cv2.imshow("DIFF", frame_delta)

    thresh = cv2.threshold(frame_delta, 30, 255, cv2.THRESH_BINARY)[1]

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
    # if len(figures) > 0:
    #    rememberFrame = None

    target = None
    for fig in figures:
        fig['delta_x'] = (fig['Pos_x'] + (fig['width'] / 2)) - (config["resolution"][0] / 2)
        fig['delta_y'] = (fig['Pos_y'] + (fig['height'] / 2)) - (config["resolution"][1] / 2)
        fig['distance'] = math.sqrt(fig['delta_x'] ** 2 + fig['delta_y'] ** 2)
        if target is None:
            target = fig
        else:
            # if target['distance'] < fig['distance']:
            if abs(target["delta_x"] * target["delta_y"]) < abs(fig["delta_x"] * fig["delta_y"]):
                target = fig
                # cv2.rectangle(image, (fig['Pos_x'], fig['width']), (fig['Pos_x'] + fig['Pos_y'], fig['width']+ fig['height']), (0, 255, 0), 2)
    if target is not None:
        print(target)
        if abs(target['delta_x']) > config['delta'] or abs(target['delta_y']) > config['delta']:
            # if abs(target['delta_x']) < config['delta']: target['delta_x'] = 0.0
            # if abs(target['delta_y']) < config['delta']: target['delta_y'] = 0.0
            angle_x = (target['delta_x'] / config["resolution"][0]) * config['angle_view'][0]
            angle_y = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]

            movement.laser_on()
            inBoundries = movement.move(angle_x, angle_y, multiplier=10)
            if inBoundries is False:
                movement.center()
            movement.delay(6000)
            movement.laser_off()
            return None

    return gray_image
