#!/usr/bin/env python3
# by cenkt

import numpy as np
import rospy
from simple_cartographer import Cartographer
from simple_node import CommunicativeNode
from simple_robot import Robot


class SphericalScanner(CommunicativeNode):
    def __init__(self):
        super(SphericalScanner, self).__init__("spherical_scanner")
        self.init_publisher()
        self.init_execution_publisher()
        self.wait_for_connections()
        self.cartographer = Cartographer()
        # Approximate target point to scan around
        self.target = np.array([0.3, 0, 0.00125])

    def scan(self):
        rospy.set_param("end_effector", 9)
        last_pose = Robot().get_current_pose(9)

        target = np.array([0.30, 0, 0.00125])
        i = 0
        for phi in np.linspace(-0.9 * np.pi / 2, 0.9 * np.pi / 2, 4):
            for theta in np.linspace(np.pi / 8, np.pi / 3.25, 4):
                for r in np.linspace(0.4, 0.55, 3):
                    rospy.loginfo(f"SC: Heading to pose #{i}")
                    peak_point = self.cartographer.spherical_to_cartesian(target, r, theta, phi)
                    T = self.cartographer.look_at_the_point(last_pose, peak_point, target)

                    self.publish(T)

                    msg = None
                    while msg != "next":
                        rospy.loginfo(f"SC: Waiting for execution")
                        msg = self.wait_for_execution()

                    rospy.sleep(1)
                    last_pose = T
                    i = i + 1
        self.execution_publisher.publish("scanning_done")


if __name__ == "__main__":
    scanner = SphericalScanner()
    scanner.scan()
