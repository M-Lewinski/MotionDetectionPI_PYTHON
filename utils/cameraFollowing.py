import cv2
import copy
import math
import datetime
import time
from utils.fps import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
from utils.motion import Movement


def calculate_target(figures, resolution_x, resolution_y):
    target = None
    for fig in figures:
        # fig['delta_x'] = (fig['Pos_x'] + (fig['width'] / 2)) - (config["resolution"][0] / 2)
        fig['delta_x'] = (fig['Pos_x'] + (fig['width'] / 2)) - (resolution_x / 2)
        # fig['delta_y'] = (fig['Pos_y'] + (fig['height'] / 2)) - (config["resolution"][1] / 2)
        fig['delta_y'] = (fig['Pos_y'] + (fig['height'] / 2)) - (resolution_y / 2)
        fig['distance'] = math.sqrt(fig['delta_x'] ** 2 + fig['delta_y'] ** 2)
        if target is None:
            target = fig
        else:
            # if target['distance'] < fig['distance']:
            if abs(target["delta_x"] * target["delta_y"]) < abs(fig["delta_x"] * fig["delta_y"]):
                target = fig
        # cv2.rectangle(image, (fig['Pos_x'], fig['width']), (fig['Pos_x'] + fig['Pos_y'], fig['width']+ fig['height']), (0, 255, 0), 2)
    return target


def track_object(image, center, tracker, config, movement: Movement):
    ok, bbox = tracker.update(image)
    figures = []
    if ok:
        pos_x = int(bbox[0])
        pos_y = int(bbox[1])
        width = abs(pos_x - int(bbox[2]))
        height = abs(pos_y - int(bbox[3]))
        figures.append({'Pos_x': pos_x, 'Pos_y': pos_y, 'width': width, 'height': height})
    else:
        return False, center
    target = calculate_target(figures, config["resolution"][0], config["resolution"][1])
    if target is not None:
        print(target)
        if abs(target['delta_x']) > config['delta'] or abs(target['delta_y']) > config['delta']:
            # if abs(target['delta_x']) < config['delta']: target['delta_x'] = 0.0
            # if abs(target['delta_y']) < config['delta']: target['delta_y'] = 0.0
            angle_x = (target['delta_x'] / config["resolution"][0]) * config['angle_view'][0]
            angle_y = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]

            center = False
            inBoundries = movement.move(angle_x, angle_y, multiplier=10)
            if inBoundries is False:
                movement.center()
                center = True
            movement.laser_on()
            movement.delay(6000)
            movement.laser_off()
            movement.delay(6000)
    return True, center


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

    rememberFrame = None
    fpsCounter = FPS()
    startTime = None
    endTime = None

    bbox = None
    tracker = cv2.TrackerKCF_create()
    center = True
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        # fps = fpsCounter.fps()
        # fpsCounter.draw_text(image, "{:.3f}".format(fps), 30, 30)
        if (bbox is None):
            bbox = cv2.selectROI(image, False)
            ok = tracker.init(image, bbox)
            if not ok:
                exit(666)
        # if startTime is None:
        #     startTime = datetime.datetime.now()
        # else:
        #     endTime = datetime.datetime.now()

        # rememberFrame = findMotion(image, rememberFrame, config, movement)
        foundTarget, center = track_object(image, center, tracker, config, movement)
        # if rememberFrame is None:
        #     startTime = None
        #     endTime = None
        # if endTime is not None:
        if center is False:
            if foundTarget is False:
                if startTime is None:
                    startTime = datetime.datetime.now()
                if (endTime - startTime).total_seconds() >= 5.0:
                    movement.center()
                    startTime = None
                    endTime = None
        else:
            startTime = None
            endTime = None
        if config['show_video'] is True:
            cv2.imshow("Primary", image)
        rawCapture.truncate(0)  # Clear capture for the next frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    movement.clean_up()
    cv2.destroyAllWindows()


def find_motion(image, rememberFrame, config, movement: Movement):
    image_cpy = copy.copy(image)
    grayImage = cv2.cvtColor(image_cpy, cv2.COLOR_BGR2GRAY)
    grayImage = cv2.GaussianBlur(grayImage, (21, 21), 0)
    if rememberFrame is None:
        # return grayImage.copy().astype("float"), image
        return grayImage
    # cv2.accumulateWeighted(grayImage, rememberFrame, 0.5)
    # frameDelta = cv2.absdiff(grayImage, cv2.convertScaleAbs(rememberFrame))
    frameDelta = cv2.absdiff(grayImage, rememberFrame)
    # cv2.imshow("DIFF", frameDelta)

    thresh = cv2.threshold(frameDelta, 30, 255, cv2.THRESH_BINARY)[1]

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
            target['angle_x'] = (target['delta_x'] / config["resolution"][0]) * config['angle_view'][0]
            target['angle_y'] = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]

            inBoundries = movement.move(angle_x, angle_y, multiplier=10)
            if inBoundries is False:
                movement.center()
            movement.laser_on()
            movement.delay(6000)
            movement.laser_off()
            movement.delay(6000)
            return None

    return grayImage
