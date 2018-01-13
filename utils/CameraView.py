import cv2
from threading import Thread

class CameraView:
    def __init__(self,config):
        self.viewDict = {}
        self.stopped = False
        self.config = config

    def start(self):
        return self

    def update(self):
        while True:
            cloneDict = self.viewDict.copy()
            for key, image in cloneDict.items():
                cv2.imshow(key, image)
            if self.stopped:
                cv2.destroyAllWindows()
                break

    def show_image(self, name, image):
        if self.config["show_video"] is True:
            cv2.imshow(name,image)

    def stop(self):
        self.stopped = True
