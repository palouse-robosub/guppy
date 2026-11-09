# guppy_vision

This package houses all of the main code to drive the vision system on Guppy.

## AprilTag Recognition

Before we train a YOLO model, we are using AprilTag detection instead of emoji glyphs. This allows us to test quickly without needing the full pipeline.

ALl the `apriltag` node does is publish four corners and fiducial ID, it does not perform any pose estimation itself.

## Webcam Publisher

This node simply publishes USB webcams as a ROS image topic. This is intended as a temporary measure until the Flir cameras are mounted externally, which have their own code.

## Sign Detection

Uses YOLO Object Detection to recognize task symbols in the pool (currently trained for 2026 tasks). Uses localized thresholding to then find the border of these signs and determine their framespace coordinates for PnP relative positioning.

## PnP Algorithm

This node uses a simple PnP approach to pose estimation from YOLO or AprilTag detections.

## Psuedo-SLAM Node

This node publishes tf2 transforms and analyzes the current images in sight. Combined with dead-reckoning odometry data from the DVL and IMU, it also generates a full global pose over time.
