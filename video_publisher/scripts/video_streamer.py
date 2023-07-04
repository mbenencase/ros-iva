#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
import argparse
import time


class VideoStreamer(object):
    def __init__(self, topic_name: str, video_src, freq: float):
        rospy.init_node("image_subscriber_node", anonymous=True)
        rospy.loginfo("Initializing ImageSubscriber node.")

        self.video_capture = cv2.VideoCapture(video_src)

        # Giving time to time
        rospy.logwarn("Wait 1.5 seconds before starting the node.")
        time.sleep(1.5)

        assert self.video_capture.isOpened()

        self.freq = freq
        self.publisher = rospy.Publisher(topic_name, CompressedImage, queue_size=1)
        self.publish_image()

    def publish_image(self):
        rate = rospy.Rate(self.freq)
        while not (rospy.is_shutdown()):
            success, frame = self.video_capture.read()
            timestamp = rospy.get_rostime()

            if not (success) or (frame is None):
                rospy.logwarn("Frame could not be read properly.")
                continue
            else:
                rospy.loginfo("Publishing frame.")

            img_msg = CompressedImage()
            img_msg.format = "jpeg"
            img_msg.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
            img_msg.header.stamp = timestamp

            self.publisher.publish(img_msg)
            rate.sleep()

        rospy.loginfo("Killing node.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", type=str, default="/camera/image_raw")
    parser.add_argument("--video-src", required=True)
    parser.add_argument(
        "--freq",
        type=float,
        default=15.0,
        help="Frequency of the image to be published.",
    )
    params = parser.parse_args()

    VideoStreamer(topic_name=params.topic, video_src=params.video_src, freq=params.freq)
