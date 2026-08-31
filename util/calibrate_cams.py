#!/bin/env python3
import cv2 as cv
import os
import numpy as np
import sys
import time
import yaml

os.environ["OPENCV_LOG_LEVEL"] = "OFF"
os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "-8"

if len(sys.argv) == 1:
    print(f"usage: {sys.argv[0]} <camera_index>")
    exit(0)

cap = cv.VideoCapture(int(sys.argv[1]))

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
while count < 150:
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
out = {"k": mtx.reshape(-1).tolist(), "d": dist[0].tolist()}
# print(out)
print(yaml.dump(out))
# # print('rmse:', ret)
# # print('camera matrix:\n', mtx)
# # print('distortion coeffs:', dist)
# # print('Rs:\n', rvecs)
# # print('Ts:\n', tvecs)

# mtx, dist
