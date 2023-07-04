#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
import torch
import time

from detector import Detector


class ObjectDetectionNode:
    def __init__(self, topic_name: str):
        rospy.init_node("object_detection_node", anonymous=True)
        rospy.loginfo("Initializing ObjectDetectionNode node.")

        self.detector = Detector()

        self.subscriber = rospy.Subscriber(
            topic_name, CompressedImage, self.img_callback, queue_size=1
        )
        rospy.spin()

    def img_callback(self, compressed_img_msg: CompressedImage) -> None:
        rospy.loginfo("Received new image.")

        img = np.frombuffer(compressed_img_msg.data, np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        t1 = time.perf_counter()
        preds = self.detector.detect(img[:, :, ::-1])
        t2 = time.perf_counter()
        spent = (t2 - t1) * 1000.0

        rospy.loginfo(f"Model take {spent:.2f}[ms] to run.")


if __name__ == "__main__":
    while not rospy.is_shutdown():
        ObjectDetectionNode(topic_name="/camera/image_raw")
