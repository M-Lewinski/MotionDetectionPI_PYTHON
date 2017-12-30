import copy
import cv2
import math
import datetime
import numpy as np


def findMotion(image, rememberFrame, config,currentFrame, frameCount, summary):
    target = None
    image_cpy = copy.copy(image)
    grayImage = cv2.cvtColor(image_cpy, cv2.COLOR_BGR2GRAY)
    grayImage = cv2.GaussianBlur(grayImage, (21, 21), 0)
    if rememberFrame is None:
        return grayImage,target,summary
    frameDelta = cv2.absdiff(grayImage, rememberFrame)
    thresh = cv2.threshold(frameDelta, 30, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    if summary is None:
        summary = np.array(thresh)
    else:
        summary = summary + np.array(thresh)
    if currentFrame < frameCount:
        return rememberFrame, target,summary
    summary = np.array(summary)/frameCount
    cv2.imshow("DIFF", summary)

    (_, contours, _) = cv2.findContours(summary.astype(np.uint8).copy(), cv2.RETR_EXTERNAL,
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
    print(figures)
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
        # print(target)
        # if abs(target['delta_x']) > config['delta'] or abs(target['delta_y']) > config['delta']:
        # if abs(target['delta_x']) < config['delta']: target['delta_x'] = 0.0
        # if abs(target['delta_y']) < config['delta']: target['delta_y'] = 0.0
        angle_x = (target['delta_x'] / config["resolution"][0]) * config['angle_view'][0]
        angle_y = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]
        return None, target, summary
    return grayImage, target, summary




def TrackingTest2(config):
    rememberFrame = None
    startTime = None
    camera = cv2.VideoCapture(0)
    # tracker = cv2.TrackerKCF_create()
    tracker = cv2.TrackerMedianFlow_create()
    bbox = None
    summary = None
    currentCount = 0
    frameCount = 5
    while True:
        ok, image = camera.read()
        if bbox is None:
            rememberFrame, target, summary = findMotion(image, rememberFrame, config,currentCount,frameCount,summary)
            currentCount += 1
            if target is not None:
                bbox = (target['Pos_x'],target['Pos_y'],target['Pos_x']+target['width'],target['Pos_y']+target['height'])
                tracker = cv2.TrackerMedianFlow_create()
                tracker.init(image, bbox)
                print(bbox)
                startTime = None
        if bbox is not None:
            ok, bbox = tracker.update(image)
            if ok:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(image, p1, p2, (255, 0, 0), 2, 1)
                startTime = None
            else:
                if startTime is None:
                    startTime = datetime.datetime.now()
                endTime = datetime.datetime.now()
                if endTime is not None and startTime is not None:
                    if (endTime - startTime).total_seconds() >= 5.0:
                        summary = None
                        currentCount = 0
                        startTime = None
                        rememberFrame = None
                        bbox = None
                cv2.putText(image, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        if config['show_video'] is True:
            cv2.imshow("Primary", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    camera.release()
    cv2.destroyAllWindows()


def TrackingTest():
    camera = cv2.VideoCapture(0)
    tracker = cv2.TrackerKCF_create()
    ok, frame = camera.read()
    bbox = None
    while True:
        ok, frame = camera.read()
        if bbox is None:

            tracker.init(frame, bbox)
            print(bbox)
        if not ok:
            break
        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break
    cv2.destroyAllWindows()
    camera.release()