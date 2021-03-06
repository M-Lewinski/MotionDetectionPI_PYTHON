import copy
import cv2
import math
import datetime
import numpy as np
from utils.CameraView import CameraView
import threading

def rect_distance(fig1,fig2):
    left = fig2['bottom_right_x'] < fig1['top_left_x']
    right = fig1['bottom_right_x'] < fig2['top_left_x']
    bottom = fig2['bottom_right_y'] < fig1['top_left_y']
    top = fig1['bottom_right_y'] < fig2['top_left_y']
    if top and left:
        p1 = np.array([fig1['top_left_x'],fig1['bottom_right_y']])
        p2 = np.array([fig2['bottom_right_x'],fig2['top_left_y']])
        return np.linalg.norm(p1 - p2)
    elif left and bottom:
        p1 = np.array([fig1['top_left_x'], fig1['top_left_y']])
        p2 = np.array([fig2['bottom_right_x'], fig2['bottom_right_y']])
        return np.linalg.norm(p1 - p2)
    elif bottom and right:
        p1 = np.array([fig1['bottom_right_x'], fig1['top_left_y']])
        p2 = np.array([fig2['top_left_x'], fig2['bottom_right_y']])
        return np.linalg.norm(p1 - p2)
    elif right and top:
        p1 = np.array([fig1['bottom_right_x'], fig1['bottom_right_y']])
        p2 = np.array([fig2['top_left_x'], fig2['top_left_y']])
        return np.linalg.norm(p1 - p2)
    if left:
        return fig1['top_left_x'] - fig2['bottom_right_x']
    elif right:
        return fig2['top_left_x'] - fig1['bottom_right_x']
    elif bottom:
        return fig1['top_left_y'] - fig2['bottom_right_y']
    elif top:
        return fig2['top_left_y'] - fig1['bottom_right_y']
    else:             # rectangles intersect
        return 0

def connect_figures(figures):
    if len(figures) < 1:
        return figures
    for fig in figures:
        fig['top_left_x'] = fig['Pos_x']
        fig['top_left_y'] = fig['Pos_y']
        fig['bottom_right_x'] = fig['Pos_x'] + fig['width']
        fig['bottom_right_y'] = fig['Pos_y'] + fig['height']
    id = 0
    lenght = len(figures)
    connected = np.zeros(lenght).astype(int) + (lenght + 1)
    min_dist = 30
    for i in range(lenght):
        if connected[i] == (lenght + 1):
            connected[i] = id
            id = id + 1
        if i < lenght - 1:
            for j in range(i+1, lenght):
                dist_between_rects = rect_distance(figures[i], figures[j])
                if dist_between_rects < min_dist:
                    val = min(connected[i], connected[j])
                    connected[i] = connected[j] = val
    maximum = connected.max() + 1
    new_figures = []
    for i in range(maximum):
        positions = np.where(connected==i)[0]
        if len(positions) > 0:
            same_figures = [[],[]]
            for pos in positions:
                fig = figures[pos]
                same_figures[0].append(fig['top_left_x'])
                same_figures[0].append(fig['bottom_right_x'])
                same_figures[1].append(fig['top_left_y'])
                same_figures[1].append(fig['bottom_right_y'])
            top_left_x = min(same_figures[0])
            top_left_y = min(same_figures[1])
            bottom_right_x = max(same_figures[0])
            bottom_right_y = max(same_figures[1])
            new_fig = {'Pos_x': top_left_x, 'Pos_y': top_left_y, 'width': abs(top_left_x - bottom_right_x), 'height': abs(top_left_y - bottom_right_y)}
            new_figures.append(new_fig)
    return new_figures

def GetChannel(image,rememberFrame,channel_number,result):
    image_array = np.array(image)
    remember_array = np.array(rememberFrame)
    remember_channel = remember_array[:,:,channel_number]
    channel = image_array[:,:,channel_number]
    difference = cv2.absdiff(channel,remember_channel)
    difference = cv2.GaussianBlur(difference,(11,11),0)
    difference = cv2.dilate(difference, (3,3), iterations=2)
    result[channel_number] = difference

def Thread_get_channel(image,rememberFrame):
    result = [None]*3
    threads = []
    for i in [1,2]:
        t = threading.Thread(target=GetChannel, args=(image,rememberFrame,i,result))
        t.start()
        threads.append(t)
    for i in range(len(threads)):
        threads[i].join()
    return (result[1] + result[2])//2

def findMotion(image, rememberFrame, config, current_frame, frame_count, summary, view : CameraView):
    target = None
    image_cpy = copy.copy(image)
    if rememberFrame is None:
        return image,target,summary
   # frameDelta = GetChannel(image_cpy,rememberFrame,2)
   # for i in range(1,2):
   #     frameDelta = (frameDelta+GetChannel(image_cpy,rememberFrame,i))//2
    frameDelta = Thread_get_channel(image,rememberFrame)  
    view.show_image("absulate diff", frameDelta)
    thresh = cv2.threshold(frameDelta,50, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    if summary is None:
        summary = thresh
    else:
        summary = cv2.bitwise_or(summary,thresh)
    if current_frame < frame_count:
        return rememberFrame, target,summary
    view.show_image("summary", frameDelta)
    (_, contours, _) = cv2.findContours(summary.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
    figures = []
    for c in contours:
        if cv2.contourArea(c) < config["min_area"]:
            continue
        pos_x, pos_y, width, height = cv2.boundingRect(c)
        figures.append({'Pos_x': pos_x, 'Pos_y': pos_y, 'width': width, 'height': height})
    figures = connect_figures(figures)
    for fig in figures:
        fig['delta_x'] = (fig['Pos_x'] + (fig['width'] / 2)) - (config["resolution"][0] / 2)
        fig['delta_y'] = (fig['Pos_y'] + (fig['height'] / 2)) - (config["resolution"][1] / 2)
        fig['distance'] = math.sqrt(fig['delta_x'] ** 2 + fig['delta_y'] ** 2)
        fig['area'] = fig['width'] * fig['height']
        if target is None:
            target = fig
        else:
            if target['area'] < fig['area']:
                target = fig
        #cv2.rectangle(image, (fig['Pos_x'], fig['Pos_y']), (fig['Pos_x'] + fig['width'], fig['Pos_y']+ fig['height']), (0, 255, 255), 2)
    if target is not None:
        if abs(target['delta_x']) > config['delta'] or abs(target['delta_y']) > config['delta']:
            # if abs(target['delta_x']) < config['delta']: target['delta_x'] = 0.0
            # if abs(target['delta_y']) < config['delta']: target['delta_y'] = 0.0
            target['angle_x'] = (target['delta_x'] / config["resolution"][0]) * config['angle_view'][0]
            target['angle_y'] = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]
            cv2.rectangle(image, (target['Pos_x'], target['Pos_y']),
                          (target['Pos_x'] + target['width'], target['Pos_y'] + target['height']),
                          (0, 255, 0), 2)
        else:
            target = None
        return None, target, summary
    return image, target, summary



def TrackingTest2(config):
    camera = cv2.VideoCapture(0)
    remember_frame = None
    start_time = None
    summary = None
    current_count = 0
    frame_count = 3
    found = False
    show_video = CameraView(config).start()
    while True:
        if(current_count == 0):
            # remember_frame = None
            summary = None
        ok, image = camera.read()
        remember_frame, target, summary = findMotion(image, remember_frame, config,current_count,frame_count,summary,show_video)
        current_count += 1
        # if remember_frame is None:
        #     remember_frame = new_frame
        if target is not None:
            remember_frame = None
            start_time = None
            summary = None
            current_count = 0
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
        show_video.show_image("Primary", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    camera.release()
    cv2.destroyAllWindows()
