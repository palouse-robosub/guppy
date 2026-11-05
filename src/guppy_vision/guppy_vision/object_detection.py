#!/usr/bin/env python3

import json as jsonlib

import cv2 as cv
import rclpy
import numpy as np
import supervision as sv
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from ultralytics import YOLO
from vision_msgs.msg import Point2D

from guppy_msgs.msg import CornerDetection, CornerDetectionList


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")

        # Model generously provided by OSU-UWRT
        self.model = YOLO("/home/andy/code/guppy/src/guppy_vision/resource/ffc_rs_26.pt")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            CompressedImage, "/cam/test/compressed", self.callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            CornerDetectionList, "/cam/test/detections", 10
        )
        self.anno_pub = self.create_publisher(
            Image, "/cam/test/anno", qos_profile_sensor_data
        )

        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        results = self.model(frame)

        corner_list = CornerDetectionList()

        boxes = results[0].boxes if results[0].boxes is not None else None

        if boxes:
            json = jsonlib.loads(results[0].to_json())
            for i, det in enumerate(boxes.xyxy.tolist()):
                newDet = CornerDetection()

                x1, y1, x2, y2 = map(int, det)

                w = x2 - x1
                h = y2 - y1

                # offset from the detection to grab whole sign
                off_x = int(w * 0.3)
                off_y = int(h * 0.3)

                off = max(
                    off_x, off_y
                )  # grab highest for detections not close to 1:1

                # clamp to frame
                y1_bound = max(0, y1 - off)
                x1_bound = max(0, x1 - off)
                y2_bound = min(frame.shape[0], y2 + off)
                x2_bound = min(frame.shape[1], x2 + off)

                dst = frame[y1_bound:y2_bound, x1_bound:x2_bound].copy()

                gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)

                # blur
                blur = cv.GaussianBlur(gray, (5, 5), 0)

                # threshold
                _, mask = cv.threshold(
                    blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU
                )

                # outlines of white sections
                cnts, _ = cv.findContours(
                    mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
                )

                # grab biggest contour
                c = max(cnts, key=cv.contourArea)
                peri = cv.arcLength(c, True)

                # reduce to 4 points
                approx = cv.approxPolyDP(c, 0.04 * peri, True)
                if len(approx) < 4: continue
                nsides = cv.approxPolyN(approx, 4)

                cntOffset = nsides + (x1_bound, y1_bound)

                for x, y in nsides.reshape(-1, 2):
                    point = Point2D()
                    point.x = float(x)
                    point.y = float(y)
                    newDet.corners.append(point)

                # for debugging
                self.get_logger().debug(cntOffset.tolist())

                # annotate frame
                cv.drawContours(frame, [cntOffset], -1, (0, 255, 0), 2)
                for x, y in cntOffset.reshape(-1, 2):
                    cv.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

                # yolo output
                newDet.confidence = json[i]["confidence"]
                newDet.name = json[i]["name"]

                # for slam - square sign of known dimensions
                newDet.square = True

                points = [
                    (-0.08, 0.08, 0),
                    (0.08, 0.08, 0),
                    (0.08, -0.08, 0),
                    (-0.08, -0.08, 0),
                ]

                for x, y, z in points:
                    point = Point()
                    point.x = float(x)
                    point.y = float(y)
                    point.z = float(z)
                    newDet.dimension_points.append(point)

                corner_list.detections.append(newDet)


            dets = sv.Detections.from_ultralytics(results[0])
            frame = self.box_annotator.annotate(scene=frame, detections=dets)
            frame = self.label_annotator.annotate(
                scene=frame, detections=dets
            )

        corner_list.header = msg.header
        self.pub.publish(corner_list)

        self.anno_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    objdet = ObjectDetection()
    rclpy.spin(objdet)
    objdet.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
