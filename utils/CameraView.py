import cv2
from threading import Thread
from threading import RLock

class CameraView:
    def __init__(self,config):
        self.viewDict = {}
        self.stopped = False
        self.config = config
        self.lock = RLock()

    def start(self):
        if self.config['show_video'] is True:
            Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            self.lock.acquire()
            cloneDict = self.viewDict.copy()
            self.lock.release()
            for key, image in cloneDict.items():
                cv2.imshow(key, image)
            if self.stopped:
                cv2.destroyAllWindows()
                break

    def show_image(self, name, image):
        self.lock.acquire()
        self.viewDict[name] = image
        self.lock.release()
        
    def stop(self):
        self.stopped = True
