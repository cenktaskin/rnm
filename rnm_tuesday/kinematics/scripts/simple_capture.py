#!/usr/bin/env python3
# by cenkt

import pickle
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from simple_node import CommunicativeNode
from simple_robot import Robot


class ImageCapture(CommunicativeNode):
    def __init__(self, node_name):
        super(ImageCapture, self).__init__(node_name)
        self.bridge = CvBridge()
        self.robot = Robot()
        self.init_execution_publisher()
        self.init_subscriber(self.j_states_topic, JointState, self.robot.callback)
        self.data = []
        self.ongoing_flag = True

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        return img

    def idle(self):
        msg = None
        while msg != "reached":
            msg = self.wait_for_execution()

            if msg == 'scanning_done':
                self.ongoing_flag = False

        rospy.sleep(1)

        # Vision part didn't ask for ir images so they are commented out
        # ir_msg = rospy.wait_for_message(self.ir_topic, Image)
        rgb_msg = rospy.wait_for_message(self.rgb_topic, Image)

        # ir_img = self.callback(ir_msg)
        rgb_img = self.callback(rgb_msg)

        pose = self.robot.get_current_pose(8)

        # bundle = [rgb_img, ir_img, pose]
        bundle = [rgb_img, pose]
        self.data.append(bundle)
        self.execution_publisher.publish("next")
        rospy.loginfo(f"CC: Published the request for next pose")

    def save_on_exit(self):
        rospy.logwarn(f"Saving images...")
        output_dir = "/home/cenkt/rnm/files/scanning_output/"
        with open(output_dir + "scanning.pkl", "wb") as f:
            pickle.dump(self.data, f)
        rospy.logwarn(f"Process done")


if __name__ == '__main__':
    rospy.set_param("end_effector", 9)
    ic = ImageCapture("simple_capture_node")

    while ic.ongoing_flag:
        try:
            ic.idle()
        except rospy.exceptions.ROSInterruptException:
            ic.save_on_exit()
