#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from simple_cartographer import Cartographer
from simple_ik import InverseKinematics
from simple_node import CommunicativeNode
from simple_robot import Robot


class Inserter(CommunicativeNode):
    def __init__(self, node_name):
        super(Inserter, self).__init__(node_name)
        self.robot = Robot()
        self.peak_pose = None
        self.cartographer = Cartographer()
        # Since vision part didn't provide a goal pose, a hand-measured target is defined
        # self.target = rospy.wait_for_message("/target_point", Float64MultiArray).data
        self.target = None

    def set_target(self, target):
        self.target = target

    def peak_at_target(self, predefined_peak_point=None):
        self.init_publisher()
        self.wait_for_connections()
        current_pose = self.robot.get_current_pose(10)

        # Vision part didn't define target or approaching point, so it can be defined manually
        if predefined_peak_point:
            peak_point = predefined_peak_point
        else:
            peak_point = current_pose[:-1, -1]

        self.peak_pose = self.cartographer.look_at_the_point(current_pose, peak_point, self.target)
        self.wait_for_connections()
        self.publish(self.peak_pose)

    def insert(self):
        # Define amount of intermediate points relative to cartesian distance
        dist = np.sqrt(np.sum((self.peak_pose[:-1, -1] - self.target) ** 2))
        intermediate_points = int(dist * 100)

        line = np.linspace(self.peak_pose[:-1, -1], self.target, intermediate_points)
        self.init_publisher(self.command_topic)
        self.wait_for_connections()

        insertion_plan = [self.get_joint_states()]
        goal = self.peak_pose
        # Get j_states for each intermediate point
        for i in range(intermediate_points - 1):
            goal[:-1, -1] = line[i + 1].T
            ik = InverseKinematics(goal=goal, initial_pose=insertion_plan[-1], end_eff=10)
            next_j_states = ik.calculate_joint_parameters()
            insertion_plan.append(next_j_states)

        rospy.logwarn("IN: Calculation done")

        # Expand j_states list into a plan
        final_plan = []
        for i in range(len(insertion_plan) - 1):
            final_plan.append(np.linspace(insertion_plan[i], insertion_plan[i + 1], 1000))
        final_plan = np.array(final_plan).reshape(-1, 7)
        rospy.logwarn(f"IN: Published plan with shape {final_plan.shape}")
        self.publish(final_plan)
        rospy.sleep(2)


if __name__ == "__main__":
    insertion_node = Inserter('insertion_node')
    rospy.set_param("end_effector", 10)

    # Comment out the line below when there is target point topic working
    insertion_node.set_target(np.array([0.335, 0.065, 0.09]))
    insertion_node.peak_at_target(predefined_peak_point=np.array([0.35, -0.25, 0.5]))
    insertion_node.wait_for_execution()
    rospy.sleep(3)
    insertion_node.insert()
