import cv2
from motion import Movement
import math

def calculateTarget(figures,resolution_x, resolution_y):
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

def trackObject(image,center,tracker,config,movement: Movement):
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
    target = calculateTarget(figures,config["resolution"][0],config["resolution"][1])
    if target is not None:
        print(target)
        if abs(target['delta_x']) > config['delta'] or abs(target['delta_y']) > config['delta']:
            # if abs(target['delta_x']) < config['delta']: target['delta_x'] = 0.0
            # if abs(target['delta_y']) < config['delta']: target['delta_y'] = 0.0
            angle_x = (target['delta_x']/config["resolution"][0])*config['angle_view'][0]
            angle_y = (target['delta_y'] / config["resolution"][1]) * config['angle_view'][1]

            center = False
            inBoundries = movement.move(angle_x,angle_y,multiplier=10)
            if inBoundries is False:
                movement.center()
                center = True
            movement.laser_on()
            movement.delay(6000)
            movement.laser_off()
            movement.delay(6000)
    return True,center
