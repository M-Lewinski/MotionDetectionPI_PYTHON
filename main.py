import argparse
import json

from utils.camera import camera_control

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--conf", default="json.conf",
                    help="path to the JSON configuration file, default: ./json.conf")
    args = vars(ap.parse_args())
    conf = json.load(open(args["conf"]))
    camera_control(conf, conf.get('debug', False))
