import argparse
import json
from motion import cameraControl



if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-c","--conf",default="json.conf",help="path to the JSON configuration file, default: ./json.conf")
    args = vars(ap.parse_args())
    conf = json.load(open(args["conf"]))
    cameraControl(conf)
