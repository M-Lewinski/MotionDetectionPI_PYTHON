import datetime

import cv2


class FPS:
    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0
        self.start()

    def start(self):
        # start the timer
        if self._start is None:
            self._start = datetime.datetime.now()
        return self

    def end(self):
        # check end time
        self._end = datetime.datetime.now()

    def update(self):
        # increment the total number of frames examined during the start and end intervals
        self._numFrames += 1

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start).total_seconds()

    def fps(self):
        self.update()
        self.end()
        seconds = self.elapsed()
        fps = self._numFrames / self.elapsed()
        if seconds > 1:
            self._start = self._end
            self._numFrames = 0
        # compute the (approximate) frames per second
        return fps

def draw_text(frame, text, x, y, color=(120, 255, 0), thickness=2, size=1):
    if x is not None and y is not None:
        cv2.putText(
            frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)
