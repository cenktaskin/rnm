#!/usr/bin/env python
import cv2 as cv
import rospy

import multiprocessing
import sys
import glob
import numpy as np
import sys
import cv2 as cv
# from cv2 import solve
import scipy
import scipy.linalg as linalg
from numpy.linalg import lstsq
import sympy
import std_msgs
from std_msgs.msg import Float64MultiArray


# The hand eye calibration is done to find the calibration of camera with respect to the
# gripper or the end-effector. Traditionally the equation AX = YB solves the equation.
# Reproduced the paper QR24 hand eye optimisation where X and Y are simultaneously solved


class hand_eye:

    def ___init___(self):
        pass

# b2g- base2gripper
# c2t camera to tracking system
# From paper e12 = identity matrix of size 12.
    def calibratehandeye(self, base2grip, cam2checker):
# counts the number of transformations in b2g
        count = len(cam2checker)
        e12 = np.eye(12, dtype=np.float64)
        e12 = e12 * -1
        z = []
# camera to gripper = c2g
# robot base to checkeboard - g2c
        grip2cam = np.eye(4, dtype=np.float64)
        base2checker = np.eye(4, dtype=np.float64)
# Aw = b , A belongs to 12n x 24 and b belongs to 12x1 vector
        A_mat_comb = np.zeros((count*12, 24), dtype=np.float64)
        B_mat_comb = np.zeros((count*12, 1), dtype=np.float64)
        A_mat = np.zeros((12, 24), dtype = np.float64)
        B_mat= np.zeros((12, 1), dtype = np.float64)

# Repeats the process for all the transformations in count
        for i in range(count):
            # forward kinematics -fk_b2g
            # split into rotation and translation
            forwardkin_b2g = base2grip[i]
            forwardkin_b2g = forwardkin_b2g.reshape(4,4)
            Rot_b2g = forwardkin_b2g[0:3, 0:3]
            Trans_b2g = -forwardkin_b2g[0:3, 3:4]
            #print(Trans_b2g)
            # camera extrinsic matrix
            cam_tf = cam2checker[i]
            cam_tf = cam_tf.reshape(4,4)
            cam_tf = np.linalg.inv(cam_tf)
            #A_ind = np.zeros((12,24), dtype = np.float64)
            #B_ind = np.zeros((12,1), dtype = np.float64)
            # Values assigned according to the paper to A and B Matrix
            for a in range(14):
                for x in range(3):
                    for y in range(3):
                        if a==0:
                            A_mat[x][y] = Rot_b2g[x][y]*cam_tf[0][0]

                        elif a==1:
                            A_mat[x+3][y] = Rot_b2g[x][y]*cam_tf[0][1]

                        elif a==2:
                            A_mat[x+6][y] = Rot_b2g[x][y]*cam_tf[0][2]

                        elif a==3:
                            A_mat[x+9][y] = Rot_b2g[x][y]*cam_tf[0][3]

                        elif a==4:
                            A_mat[x][y+3] = Rot_b2g[x][y]*cam_tf[1][0]

                        elif a==5:
                            A_mat[x+3][y+3] = Rot_b2g[x][y]*cam_tf[1][1]

                        elif a==6:
                            A_mat[x+6][y+3] = Rot_b2g[x][y]*cam_tf[1][2]

                        elif a==7:
                            A_mat[x+9][y+3] = Rot_b2g[x][y]*cam_tf[1][3]

                        elif a==8:
                            A_mat[x][y+6] = Rot_b2g[x][y]*cam_tf[2][0]

                        elif a == 9:
                            A_mat[x+3][y+6] = Rot_b2g[x][y]*cam_tf[2][1]

                        elif a == 10:
                            A_mat[x+6][y+6] = Rot_b2g[x][y]*cam_tf[2][2]

                        elif a == 11:
                            A_mat[x+9][y+6] = Rot_b2g[x][y]*cam_tf[2][3]

                        elif a == 12:
                            A_mat[x+9][y+9] = Rot_b2g[x][y]

            A_mat[0:12, 12:24] = e12
            #print(A_ind)

            for x in range(12):
                for y in range(24):
                    A_mat_comb[(i*12)+x][y] = A_mat[x][y]
            # Assigning values to B matrix
            for y in range(len(Trans_b2g)):
                B_mat[9+y] = Trans_b2g[y]

            for y in range(len(B_mat)):
                B_mat_comb[(i*12)+y] = B_mat[y]

        #print(B)
        #print(A)
        print(A_mat_comb.shape)
        # Calculating the the final transformation using least squares method
        z = lstsq(A_mat_comb, B_mat_comb)
        # Splitting 24x24 into end-effector to camera and robot to checkerboard
        grip2cam[0][0] = z[0][0]
        grip2cam[0][1] = z[0][1]
        grip2cam[0][2] = z[0][2]
        grip2cam[0][3] = z[0][3]
        grip2cam[1][0] = z[0][4]
        grip2cam[1][1] = z[0][5]
        grip2cam[1][2] = z[0][6]
        grip2cam[1][3] = z[0][7]
        grip2cam[2][0] = z[0][8]
        grip2cam[2][1] = z[0][9]
        grip2cam[2][2] = z[0][10]
        grip2cam[2][3] = z[0][11]
        print("End Effector to camera ")
        print(grip2cam)

        base2checker[0][0] = z[0][12]
        base2checker[0][1] = z[0][13]
        base2checker[0][2] = z[0][14]
        base2checker[0][3] = z[0][15]
        base2checker[1][0] = z[0][16]
        base2checker[1][1] = z[0][17]
        base2checker[1][2] = z[0][18]
        base2checker[1][3] = z[0][19]
        base2checker[2][0] = z[0][20]
        base2checker[2][1] = z[0][21]
        base2checker[2][2] = z[0][22]
        base2checker[2][3] = z[0][23]

        print("RobotBase to Checkerboard")
        print(base2checker)

        return grip2cam, base2checker


if __name__ == '__main__':
    #rospy.init_node('hand_eye')

    # These matrices are examples just for testing. When we run on the robot,we use the matrices from
    # calibrate.py for camera2checkerboard matrix and base2gripper from the transformations we got from the
    # robot in the lab
    # the base 2 gripper transformations and the images are in the hand_eye zip file which we got from lab when we took the poses.

    base2grip_matrices = []
    cam2check_matrices = []

    poses = np.genfromtxt("/media/sf_www/txtfiles/tf3.txt", delimiter = ",")
    tf_b_ee = poses
    print("Pose transformations")
    #print(tf_b_ee)
    camera_tf = np.genfromtxt("/media/sf_www/txtfiles/cam4.txt",delimiter = ",")
    tf_cam = camera_tf
    print("camera extrinsic")
    #print(tf_cam)

    abc = hand_eye()
    print(abc.calibratehandeye(tf_b_ee, tf_cam))

    HE_pub = rospy.Publisher('/Hand_Eye', Float64MultiArray, queue_size=1)
    grip, cam2checker = abc.calibratehandeye(tf_b_ee, tf_cam)
    while not rospy.is_shutdown():
        pub_data = Float64MultiArray()
        pub_data.data = grip
        HE_pub.publish(pub_data)


















