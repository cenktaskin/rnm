#!/usr/bin/env python
from __future__ import print_function

import rospy
import roslib
import cv2
import cv_bridge
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:
    A = []
    images = 0

    def __init__(self):

        self.image_pub = rospy.Publisher('image_topic2', Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/k4a/ir/image_raw', Image, self.callback)

    def callback(self, data):
        global img
        print(data.header.seq)
        if data.header.seq % 50 == 0:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")  # type: img
            self.A.append(img)

    def print_image(self):
        print("Loop here")
        for i in self.A:
            cv2.imshow("Image_window", i)
            cv2.imwrite("/home/rnm/Documents/irNew2/" + str(self.images) + ".jpg", i)
            self.images = self.images+1
            cv2.waitKey(3)


def main(args):
    ic = ImageConverter()
    rospy.init_node('depth', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    ic.print_image()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
