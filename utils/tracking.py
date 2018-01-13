import cv2
import datetime
import time
from utils.motion import Movement
from utils.cameraTest import findMotion
from utils.CameraRawCapture import PiVideoStream
from utils.CameraView import CameraView

def camera_control(config):
    # Pi configuration and calibration
    movement = Movement()
    movement.calibrate()
    video_stream = PiVideoStream(config).start()
    time.sleep(config["camera_warmup_time"])
    remember_frame = None
    start_time = None
    summary = None
    current_count = 0
    frame_count = 1
    found = False
    show_video = CameraView(config).start()
    while True:
        image = video_stream.read_frame()
        if(current_count == 0):
            # remember_frame = None
            summary = None
        remember_frame, target, summary = findMotion(image, remember_frame, config,current_count,frame_count,summary,show_video)
        current_count += 1
        # if remember_frame is None:
        #     remember_frame = new_frame
        if target is not None:
            remember_frame = None
            start_time = None
            summary = None
            current_count = 0
           #movement.laser_on()
            inBoundries = movement.move(target['angle_x'], target['angle_y'], multiplier=10)
            time.sleep(0.2)
           #movement.laser_off()
            if inBoundries is False:
                movement.center()
            else:
                found = True
        elif found is True:
            if start_time is None:
                start_time = datetime.datetime.now()
            end_time = datetime.datetime.now()
            if (end_time - start_time).total_seconds() >= 1.5:
                print("Tracking failure detected")
                #cv2.putText(image, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255),2)
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
    video_stream.stop()
    movement.clean_up()