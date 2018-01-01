import cv2
import copy
import math
import datetime
import time
import numpy as np
from utils.fps import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
from utils.motion import Movement
from utils.cameraTest import findMotion

def camera_control(config):
    # Pi configuration and calibration
    movement = Movement()
    movement.calibrate()
    camera = PiCamera()
    res = tuple(config["resolution"])
    camera.resolution = res
    camera.framerate = config["fps"]
    rawCapture = PiRGBArray(camera, size=res)
    time.sleep(config["camera_warmup_time"])

    remember_frame = None
    start_time = None
    summary = None
    current_count = 0
    frame_count = 5
    found = False
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        if(current_count == 0):
            # remember_frame = None
            summary = None
        remember_frame, target, summary = findMotion(image, remember_frame, config,current_count,frame_count,summary)
        current_count += 1
        # if remember_frame is None:
        #     remember_frame = new_frame
        if target is not None:
            remember_frame = None
            start_time = None
            summary = None
            current_count = 0
            inBoundries = movement.move(target['angle_x'], target['angle_y'], multiplier=10)
            movement.laser_on()
            movement.delay(6000)
            movement.laser_off()
            movement.delay(6000)
            if inBoundries is False:
                movement.center()
            else:
                found = True
        elif found is True:
            if start_time is None:
                start_time = datetime.datetime.now()
            end_time = datetime.datetime.now()
            if (end_time - start_time).total_seconds() >= 1.5:
                cv2.putText(image, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255),
                            2)
            if (end_time - start_time).total_seconds() >= 5.0:
                summary = None
                current_count = 0
                start_time = None
                remember_frame = None
                found = False
        current_count = current_count % (frame_count+1)
        if config['show_video'] is True:
            cv2.imshow("Primary", image)
        rawCapture.truncate(0)  # Clear capture for the next frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    movement.clean_up()
    cv2.destroyAllWindows()