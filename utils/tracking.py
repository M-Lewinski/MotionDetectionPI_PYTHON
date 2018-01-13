import cv2
import datetime
import time
from utils.motion import Movement
from utils.cameraTest import findMotion
from utils.CameraRawCapture import PiVideoStream
from utils.aws_utils import AWSPredator
from utils.CameraView import CameraView
from utils.fps import FPS

def camera_control(config):
    # Pi configuration and calibration
    movement = Movement()
    movement.calibrate()
    aws_predator = AWSPredator()
    video_stream = PiVideoStream(config).start()
    time.sleep(config["camera_warmup_time"])
    remember_frame = None
    start_time = None
    summary = None
    current_count = 0
    frame_count = 0
    found = False
    fps_counter = FPS()
    show_video = CameraView(config).start()
    while True:
        fps = fps_counter.fps()
        print(fps)
        image = video_stream.read_frame()

        if(current_count == 0):
            summary = None
        remember_frame, target, summary = findMotion(image, remember_frame, config,current_count,frame_count,summary,show_video)
        current_count += 1
        if target is not None:
            remember_frame = None
            start_time = None
            summary = None
            current_count = 0
            movement.laser_on()

            inBoundries = movement.move(target['angle_x'], target['angle_y'], multiplier=10)
            movement.laser_off()
            aws_predator.detected_movement(image)
            time.sleep(0.3)
            if inBoundries is False:
                movement.center()
                time.sleep(0.3)
            else:
                found = True
        elif found is True:
            if start_time is None:
                start_time = datetime.datetime.now()
            end_time = datetime.datetime.now()
            if (end_time - start_time).total_seconds() >= 1.5:
                print("Tracking failure detected")
            if (end_time - start_time).total_seconds() >= 5.0:
                movement.center()
                time.sleep(0.3)
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
    video_stream.stop()
    movement.clean_up()
