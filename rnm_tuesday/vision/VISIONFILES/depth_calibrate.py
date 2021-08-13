#!/usr/bin/env python
import cv2 as cv
import rospy
import numpy as np
import math
import glob
import sensor_msgs

# initializing checkerboard size and count in order to calibrate the camera for ir/depth images
#ct is to calculate the no. of images
ct = 0
checkerboardSize = (8, 5)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 1e-6)

# 3d coordinates(objject points) to 2d(image points) to convert from world frame to camera frame
obj_points = []
img_points = []
obj_p = np.zeros((checkerboardSize[0]*checkerboardSize[1], 3), np.float32)
obj_p[:, :2] = np.mgrid[0:checkerboardSize[0], 0:checkerboardSize[1]].T.reshape(-1, 2)

# Extracting images from rosbag files, converting them to gray scale and finding corners
images = glob.glob('/home/rnm/Documents/irNew/*.jpg')
for i in images:
    img = cv.imread(i)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    success, corners = cv.findChessboardCorners(gray, checkerboardSize, flags=cv.CALIB_CB_ADAPTIVE_THRESH)

    if success:
        obj_points.append(obj_p)
# append corners to image points and draw them on the image
        corners2 = cv.cornerSubPix(gray, corners, (16, 16), (-1, -1), criteria)
        img_points.append(corners2)
        ct = ct+1
        img = cv.drawChessboardCorners(gray, checkerboardSize, corners2, success)
        print(ct)
    cv.imshow('img', img)
    cv.waitKey()
cv.destroyAllWindows()

#   Calibrate camera using opencv function
print("Calibrating...")
success, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1], cameraMatrix=None,
                                                               distCoeffs=None, rvecs=None, tvecs=None,
                                                               flags=cv.CALIB_RATIONAL_MODEL, criteria=criteria)
print("Camera Calibrated", success)
print("\nCameraMatrix:\n", cameraMatrix)
print("\nDistortion Parameters:\n", dist)
print("\nRotation vectors:\n", rvecs[0], rvecs[1], rvecs[2])
print("\nTranslation Vectors:\n", tvecs[0])
