import boto3
import datetime
import json
import threading
import cv2
import os

TOPIC_ARN = 'arn:aws:sns:eu-central-1:972411866948:predator_sns'
BUCKET = 'predator-rpi-bucket'
IMAGE_UPLOAD_TIME_SEC = 5
SNS_NOTIFICATION_TIME_SEC = 300
BASE_PATH = '/tmp/'


def send_notification(subject, message):
    client = boto3.client('sns')
    client.publish(
        TopicArn=TOPIC_ARN,
        Message=message,
        Subject=subject
    )


def upload_image_s3(image_path, file_key):
    s3 = boto3.resource('s3')
    bucket = s3.Bucket(BUCKET)
    bucket.upload_file(image_path, file_key)


def save_and_upload_image(image, x):
    path = "predator_capture_{}.jpg".format(str(datetime.datetime.now()))
    os_path = "{}{}".format(BASE_PATH, path)
    cv2.imwrite(os_path, image)
    upload_image_s3(os_path, path)
    delete_image(os_path)


def delete_image(path):
    os.remove(path)


class AWSPredator:
    def __init__(self):
        self.last_s3_image_time = None
        self.last_sns_time = None

    def detected_movement(self, image):
        now = datetime.datetime.now()
        if self.last_s3_image_time is None or (now-self.last_s3_image_time).total_seconds() > IMAGE_UPLOAD_TIME_SEC:
            self.last_s3_image_time = now
            t = threading.Thread(target=save_and_upload_image, args=(image, None))
            t.start()

        if self.last_sns_time is None or (now-self.last_sns_time).total_seconds() > SNS_NOTIFICATION_TIME_SEC:
            self.last_sns_time = now
            t = threading.Thread(target=send_notification, args=("Predator detected movement!", json.dumps({
                "Message": "Predator detected movement",
                "Time": str(now),
            })))
            t.start()
