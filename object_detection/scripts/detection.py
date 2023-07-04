#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from object_detection.msg import BoundingBox, BoundingBoxes

import cv2
import numpy as np
import time

from detector import Detector


class ObjectDetectionNode:
    def __init__(self, topic_name: str):
        rospy.init_node("object_detection_node", anonymous=True)
        rospy.loginfo("Initializing ObjectDetectionNode node.")

        self.detector = Detector()
        self.classes = self.detector.classes

        self.subscriber = rospy.Subscriber(
            topic_name, CompressedImage, self.img_callback, queue_size=1
        )
        self.publisher = rospy.Publisher("/bounding_boxes", BoundingBoxes, queue_size=1)
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

        bboxes_msg = BoundingBoxes()
        bboxes_msg.header.stamp = rospy.get_rostime()
        for x1, y1, x2, y2, conf, cls_idx in preds:
            bbox_msg = BoundingBox()
            bbox_msg.x1 = x1
            bbox_msg.y1 = y1
            bbox_msg.x2 = x2
            bbox_msg.y2 = y2
            bbox_msg.conf = conf
            bbox_msg.label = self.classes[cls_idx]
            bboxes_msg.boxes.append(bbox_msg)

        self.publisher.publish(bboxes_msg)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        ObjectDetectionNode(topic_name="/camera/image_raw")
