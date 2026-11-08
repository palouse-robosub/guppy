#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rclpy
<<<<<<< Updated upstream
import supervision as sv
=======

>>>>>>> Stashed changes
from cv_bridge import CvBridge
from vision_msgs.msg import Point2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
<<<<<<< Updated upstream
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Point2D
=======
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
>>>>>>> Stashed changes

from guppy_msgs.msg import CornerDetection, CornerDetectionList


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")

        self.bridge = CvBridge()

        # Camera calibration values.
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribe to camera calibration.
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/cam/test/camera_info",
            self.camera_info_callback,
            10
        )

        # Subscribe to camera images.
        self.sub = self.create_subscription(
            Image,
            "/cam/test",
            self.callback,
            qos_profile_sensor_data
        )

        # Publish gate detections.
        self.pub = self.create_publisher(
            CornerDetectionList,
            "/cam/test/detections",
            10
        )

        # Publish annotated camera images.
        self.anno_pub = self.create_publisher(
            Image,
            "/cam/test/anno",
            qos_profile_sensor_data
        )

        # Publish estimated 3D gate position.
        self.position_pub = self.create_publisher(
            PointStamped,
            "/cam/test/gate_position",
            10
        )

        self.get_logger().info("OpenCV gate detector started.")

    def camera_info_callback(self, msg):
        # Read camera calibration values.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        # Convert the camera image to HSV.
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Find the black parts of the gate.
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180, 100, 100])
        black_mask = cv.inRange(
            hsv,
            black_lower,
            black_upper
        )

        # Find the red parts of the gate.
        red_lower1 = np.array([0, 80, 40])
        red_upper1 = np.array([10, 255, 255])

        red_lower2 = np.array([170, 80, 40])
        red_upper2 = np.array([180, 255, 255])

        red_mask1 = cv.inRange(
            hsv,
            red_lower1,
            red_upper1
        )

        red_mask2 = cv.inRange(
            hsv,
            red_lower2,
            red_upper2
        )

        red_mask = cv.bitwise_or(red_mask1, red_mask2)

        # Combine the black and red parts.
        mask = cv.bitwise_or(black_mask, red_mask)

        # Join nearby gate sections into one shape.
        kernel = np.ones((15, 15), np.uint8)

        mask = cv.morphologyEx(
            mask,
            cv.MORPH_CLOSE,
            kernel
        )

        # Find the outside contours.
        contours, _ = cv.findContours(
            mask,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE
        )

        corner_list = CornerDetectionList()
        annotated_image = frame.copy()

        best_gate = None
        best_area = 0

        # Look for a wide, gate-shaped contour.
        for contour in contours:
            area = cv.contourArea(contour)

            x, y, width, height = cv.boundingRect(contour)

            if height == 0:
                continue

            aspect_ratio = width / height

            if (
                area > 500
                and width > 100
                and height > 40
                and 1.5 < aspect_ratio < 4.5
            ):
                if area > best_area:
                    best_area = area
                    best_gate = (x, y, width, height)

        # Publish the gate detection if one was found.
        if best_gate is not None:
            x, y, width, height = best_gate

            det = CornerDetection()
            det.name = "gate"
            det.confidence = 0.85
            det.square = False

            # Four image corners, ordered around the rectangle.
            corners = [
                (x, y),
                (x + width, y),
                (x + width, y + height),
                (x, y + height)
            ]

            for px, py in corners:
                point = Point2D()
                point.x = float(px)
                point.y = float(py)
                det.corners.append(point)

            corner_list.detections.append(det)

            # Draw the detection on the camera image.
            cv.rectangle(
                annotated_image,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                3
            )

            cv.putText(
                annotated_image,
                "gate",
                (x, max(y - 10, 20)),
                cv.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            # Calculate distance and position.
            if (
                self.fx is not None
                and self.fy is not None
                and width > 0
                and height > 0
            ):
                # Known gate dimensions in meters.
                gate_width = 3.0988
                gate_height = 1.2664

                # Estimate distance using width and height.
                distance_width = (
                    self.fx * gate_width
                ) / width

                distance_height = (
                    self.fy * gate_height
                ) / height

                distance = (
                    distance_width + distance_height
                ) / 2.0

                # Find the center of the gate.
                center_x = x + width / 2.0
                center_y = y + height / 2.0

                # Calculate camera-relative position.
                position_x = (
                    (center_x - self.cx) * distance
                ) / self.fx

                position_y = (
                    (center_y - self.cy) * distance
                ) / self.fy

                # Publish the estimated 3D position.
                position_msg = PointStamped()
                position_msg.header = msg.header

                position_msg.point.x = float(position_x)
                position_msg.point.y = float(position_y)
                position_msg.point.z = float(distance)

                self.position_pub.publish(position_msg)

                # Display estimated distance.
                cv.putText(
                    annotated_image,
                    f"Distance: {distance:.2f} m",
                    (20, 35),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            else:
                cv.putText(
                    annotated_image,
                    "Waiting for camera calibration",
                    (20, 35),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

        else:
            cv.putText(
                annotated_image,
                "Gate not found",
                (20, 35),
                cv.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # Publish the detection list.
        corner_list.header = msg.header
        self.pub.publish(corner_list)

        # Publish the annotated image.
        annotated_image = np.ascontiguousarray(
            annotated_image,
            dtype=np.uint8
        )

        anno_msg = Image()
        anno_msg.header = msg.header
        anno_msg.height = annotated_image.shape[0]
        anno_msg.width = annotated_image.shape[1]
        anno_msg.encoding = "bgr8"
        anno_msg.is_bigendian = 0
        anno_msg.step = annotated_image.shape[1] * 3
        anno_msg.data = annotated_image.tobytes()

        self.anno_pub.publish(anno_msg)


def main(args=None):
    rclpy.init(args=args)

    objdet = ObjectDetection()

    try:
        rclpy.spin(objdet)
    except KeyboardInterrupt:
        pass
    finally:
        objdet.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
