import cv2
from threading import Thread


class CameraView:
    def __init__(self,config):
        self.viewDict = {}
        self.stopped = False
        self.config = config

    def start(self):
        if self.config['show_video'] is True:
            Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            for key, image in self.viewDict.items():
                cv2.imshow(key, image)
            if self.stopped:
                cv2.destroyAllWindows()
                break

    def show_image(self,name,image):
        self.viewDict[name] = image

    def stop(self):
        self.stopped = True
