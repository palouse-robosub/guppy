#!/bin/env python3
import cv2 as cv
import os
import numpy as np
import sys
import time
import yaml
import datetime

os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "-8"

if len(sys.argv) < 3 or len(sys.argv) > 4:
    print(f"usage: {sys.argv[0]} <camera_index> <camera_name> [<num_images>=50]")
    exit(0)

camera_index = int(sys.argv[1])
camera_name = sys.argv[2]
num_images = 50
if (len(sys.argv) == 4):
    num_images = int(sys.argv[3])
cap = cv.VideoCapture(camera_index)

# criteria used by checkerboard pattern detector.
# Change this if the code can't find the checkerboard
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

rows = 7  # number of checkerboard rows.
columns = 10  # number of checkerboard columns.
world_scaling = 1.0  # change this to the real world square size. Or not.

# coordinates of squares in the checkerboard world space
objp = np.zeros((rows * columns, 3), np.float32)
objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2)
objp = world_scaling * objp

# frame dimensions. Frames should be the same size.
_, frame = cap.read()
width = frame.shape[1]
height = frame.shape[0]

# Pixel coordinates of checkerboards
imgpoints = []  # 2d points in image plane.

# coordinates of the checkerboard in checkerboard world space.
objpoints = []  # 3d point in real world space

count = 0
last = time.time() + 3
while count < num_images:
    _, frame = cap.read()
    # print(frame)
    cv.imshow("img", frame)
    k = cv.waitKey(1)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # find the checkerboard
    ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)

    if ret and time.time() - last > 2:
        count += 1

        # Convolution size used to improve corner detection. Don't make this too large.
        conv_size = (11, 11)

        # opencv can attempt to improve the checkerboard coordinates
        corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
        cv.drawChessboardCorners(frame, (rows, columns), corners, ret)
        cv.imshow("img", frame)
        k = cv.waitKey(500)
        last = time.time()

        objpoints.append(objp)
        imgpoints.append(corners)


ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, (width, height), None, None
)
out = {
    "usb_camera_publisher": {
        "ros__parameters": {
            "camera_name": camera_name,
            "camera_index": camera_index,
            "camera_model_last_calibrated": datetime.datetime.now().isoformat(),
            "camera_model_n_calibration_images": num_images,
            "camera_model_d": dist[0].tolist(),
            "camera_model_k": mtx.reshape(-1).tolist()
        }
    }
}

out_path = os.path.join(
    os.path.abspath(os.path.dirname(__file__)),
    f"../src/guppy_vision/calib/{camera_name}.yaml"
)

if os.path.isfile(out_path):
    print(f"the file src/guppy_vision/calib/{camera_name}.yaml already exists...")
    if input(f"overwrite? [y/N] ").lower() != "y":
        print("not writing...")
        exit(-1)

with open(out_path, "w+") as f:
    f.write(yaml.dump(out, sort_keys=False))

print(f"calibration config written to src/guppy_vision/calib/{camera_name}.yaml")
