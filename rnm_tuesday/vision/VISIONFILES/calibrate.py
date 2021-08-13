#!/usr/bin/env python
import cv2 as cv
import rospy
import numpy as np
import math
import glob


#Writing the extrinsic matrix transformation to a text file
def writing_stuff(stuff):

    with open('/media/sf_www/txtfiles/cam4.txt', 'w') as f:
        for i in range(len(stuff)):
            mat = np.asarray(stuff[i])
            #print(mat)
            mat = mat.flatten()
            lines = []

            for j in range(len(mat)):
                a = mat[j]
                b = str(a)
                #print("str",b)
                lines.append(b)

            for line in lines:
                f.write(line)
                f.write(',')
            f.write('\n')
    print("Writing done...!!!")


no_of_images = 0
checkerboardSize = (8, 5)
img_Shape = 0
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 1e-6)
# declaring object points and image points
obj_points = []
img_points = []
obj_p = np.zeros((checkerboardSize[0]*checkerboardSize[1], 3), np.float32)
obj_p[:, :2] = np.mgrid[0:checkerboardSize[0], 0:checkerboardSize[1]].T.reshape(-1, 2)

# to find and draw checkerboard corners on each grayscale image and append those corners to image points
images = glob.glob('/media/sf_www/unpickleImages/*.jpg')
for i in images:
    img = cv.imread(i)
    img = cv.resize(img, (2048, 1536))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    success, corners = cv.findChessboardCorners(gray, checkerboardSize, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK+
                                                                              cv.CALIB_CB_NORMALIZE_IMAGE)
    if success:
        obj_points.append(obj_p)
        corners2 = cv.cornerSubPix(gray, corners, (14, 14), (-1, -1), criteria)
        img_points.append(corners2)
        img = cv.drawChessboardCorners(img, checkerboardSize, corners2, success)
        no_of_images = no_of_images+1

cv.destroyAllWindows()
print("Calibrating")
success, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1],
                                                               cameraMatrix=None, distCoeffs=None, rvecs=None, tvecs=None,
                                                               flags=cv.CALIB_RATIONAL_MODEL, criteria=criteria)
# in order to combine all the extrinsic matrices of each image we use cam_matrices_combined which will be published to be used in handeye calibration
cam_matrices_combined = []
stuff = np.zeros((48,4,4))
cam_2_checkerboard_mat = np.eye(4, dtype=np.float64)

# For each image ,the rotation and translation values obtained from calibrate camera are used to form the
# homogeneous transformation matrix
for a in range(0, no_of_images):
    r_vector = np.asarray(rvecs[a], dtype=np.float64)
    Rot_Mat = np.zeros((3,3), dtype=np.float64)
    #Rodrigues converts rotation vector into a rotation matrix
    cv.Rodrigues(r_vector, Rot_Mat, jacobian=0)

    for x in range(3):
        for y in range(3):
            cam_2_checkerboard_mat[x][y] = Rot_Mat[x][y]
    for x in range(3):
        cam_2_checkerboard_mat[x][3] = tvecs[a][x]

    stuff[a] = cam_2_checkerboard_mat
    print("camera2checkerboard",cam_2_checkerboard_mat)
    cam_matrices_combined.append(cam_2_checkerboard_mat)
    print("camera matrices combined",cam_matrices_combined[-1])
print('result')
print("****************")
print(stuff)
# print(cam_matrices_combined)
# print("Camera Calibrated", success)
print("\nCameraMatrix:\n", cameraMatrix)
print("\nDistortion Parameters:\n", dist)
writing_stuff(stuff)
#print("\nRotation vectors:\n", rvecs)
#print("\nTranslation Vectors:\n", trans_vecs[0][1])

mean_error = 0

for i in range(len(obj_points)):

    img_points2, _ = cv.projectPoints(obj_points[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(img_points[i], img_points2, cv.NORM_L2)/len(img_points2)
    mean_error += error

print("total error: {}".format(mean_error/len(obj_points)))