import argparse
import json
from utils.tracking import camera_control as Tracker
#from utils.cameraTest import TrackingTest2 as Tracker


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--conf",
                    default="json.conf",
                    help="path to the JSON configuration file, default: ./json.conf"
                    )
    args = vars(ap.parse_args())
    conf = json.load(open(args["conf"]))
    Tracker(conf)
