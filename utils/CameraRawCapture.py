from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2

class PiVideoStream
        def __init__(self,config):
            self.camera = PiCamera()
            self.camera = PiCamera()
            res = tuple(config["resolution"])
            self.camera.resolution = res
            self.camera.framerate = config["fps"]
            self.rawCapture = PiRGBArray(self.camera, size=res)
            self.stream = self.stream = self.camera.capture_continuous(self.rawCapture,
			format="bgr", use_video_port=True)
            self.frame = None
            self.stopped = False

        def start(self):
            Thread(target=self.update, args=()).start()
            return self

        def update(self):
            for f in self.stream:
                self.frame = f.array
                self.rawCapture.truncate(0)
                if self.stopped:
                    self.stream.close()
                    self.rawCapture.Close()
                    self.camera.Close()
                    return

        def read_frame(self):
            return self.frame

        def stop(self):
            self.stopped = True