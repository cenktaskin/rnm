#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from scipy import optimize
from std_msgs.msg import Float64MultiArray
from simple_node import CommunicativeNode
from simple_robot import Robot


class InverseKinematics(CommunicativeNode):
    def __init__(self, node_name=None, end_eff_index=8, **kwargs):
        super(InverseKinematics, self).__init__(node_name)
        rospy.loginfo(f"IK: Inverse kinematics started for {self.execution_target}")
        rospy.loginfo(f"IK: Listening {self.goal_topic}")
        self.robot = Robot()
        self.end_eff = end_eff_index
        # The option for predefining some arguments for debugging purposes
        if kwargs:
            self.goal = kwargs['goal']
            self.initial_pose = kwargs['initial_pose']
            rospy.loginfo(f"IK: Predefined goal received")
        else:
            self.goal = np.array(rospy.wait_for_message(self.goal_topic, Float64MultiArray).data).reshape((4, 4))
            rospy.loginfo(f"IK: Goal received from {self.goal_topic}")
            self.initial_pose = self.get_joint_states()

    #def get_initial_pose(self):
    #    return self.initial_pose

    def calculate_joint_parameters(self):
        result = optimize.least_squares(fun=self.kinematic_cost_function, x0=self.initial_pose, method='trf',
                                        ftol=1e-6,
                                        bounds=(self.robot.joints_min, self.robot.joints_max),
                                        args=[self.goal, self.end_eff])
        return result.x

    def kinematic_cost_function(self, theta, goal, end_eff):
        self.robot.calculate_tf(theta)
        diff = np.abs(self.robot.t_matrices[end_eff, :-1, :] - goal[:-1, :])
        return diff.reshape(-1)


if __name__ == "__main__":
    ik = InverseKinematics('simple_ik_node')
    ik.init_publisher("/goal_joint_states")

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        goal_joint_states = np.array(ik.calculate_joint_parameters(), dtype=np.float64)
        ik.publish(goal_joint_states)
        r.sleep()
