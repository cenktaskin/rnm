#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from simple_node import CommunicativeNode


class GoalSetter(CommunicativeNode):
    goal_pose = np.array([[0.174334, -0.06051, 0.982828, 0.4],
                          [0.972122, -0.148378, -0.181574, 0.0],
                          [0.156812, 0.987082, 0.032952, 0.6],
                          [0, 0, 0, 1]])

    vanilla_pose = np.array([[5.45595126e-01, -5.45584644e-01, -6.36131554e-01, 2.62951027e-01],
                             [-7.07103227e-01, -7.07110335e-01, -5.55622196e-06, 2.06051768e-06],
                             [-4.49812165e-01, 4.49813707e-01, -7.71580615e-01, 8.59229481e-01],
                             [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    def __init__(self):
        super(GoalSetter, self).__init__("simple_goal_node")
        goal_setter_node.init_publisher()
        goal_setter_node.wait_for_connections()

    def set_the_goal(self):
        # Resetting the pose to a pose closer to simulation starting pose for debugging purposes
        if rospy.get_param('reset', False):
            goal = self.vanilla_pose
            rospy.set_param('reset', False)
        else:
            goal = self.goal_pose

        goal_setter_node.publish(goal)
        rospy.loginfo(f"Published the goal")


if __name__ == "__main__":
    goal_setter_node = GoalSetter()
