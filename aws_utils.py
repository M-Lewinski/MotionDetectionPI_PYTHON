import boto3

TOPIC_ARN = 'arn:aws:sns:eu-central-1:972411866948:predator_sns'
BUCKET = 'predator-rpi-bucket'


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

