#!/usr/bin/env python3
# by cenkt

import rospy
import time
import numpy as np
from simple_ik import InverseKinematics
from simple_quintics import QuinticTrajectory
from simple_robot import Robot
from simple_node import CommunicativeNode


class TrajectoryPlanner(CommunicativeNode):
    def __init__(self, node_name):
        super(TrajectoryPlanner, self).__init__(node_name)
        rospy.loginfo(f"TP: Trajectory planner started for {self.execution_target}")
        self.initial_pose = None
        self.goal_joint_states = None
        self.quintics = QuinticTrajectory()
        self.robot = Robot()
        self.init_publisher(self.command_topic)

    def plan_the_trajectory(self, gas_pedal=0.1):
        sufficient_plan = None
        dur = 0.1
        while sufficient_plan is None:
            dur += 0.1
            sufficient_plan = self.trajectory_creator(dur, gas_pedal)
        return sufficient_plan

    def trajectory_creator(self, dur, gas):
        res = 1000  # 1000 Hz
        pos_eqs = np.array([self.quintics.position(t) for t in np.arange(0, dur, 1 / res)])
        coeff_neu = np.array(
            [self.quintics.find_quintic_coefficients(self.initial_pose[j], self.goal_joint_states[j], dur) for
             j in range(7)])
        v, a, j = self.quintics.get_max_values_from_traj(coeff_neu, dur)
        if self.robot.check_limits(v, a, j, gas):
            paths_neu = np.array([pos_eqs @ c for c in coeff_neu])
            return paths_neu

    def idle(self):
        end_effector = rospy.get_param('/end_effector', 8)
        gas = rospy.get_param('gas', 0.1)

        rospy.loginfo(f"TP: Using {gas} of total speed with end_effector={end_effector}")

        start_time = time.time()
        ik = InverseKinematics(end_eff_index=end_effector)
        self.initial_pose = self.get_joint_states()

        success = False
        while not success and not rospy.is_shutdown():
            try:
                self.goal_joint_states = ik.calculate_joint_parameters()
                success = True
            except ValueError:
                self.handle_singularity()

        plan = self.plan_the_trajectory(gas)
        rospy.sleep(1)
        self.publish(plan.T)
        rospy.loginfo(f"TP: Planned and published in {np.round(time.time() - start_time, 4)} secs")

    def handle_singularity(self, ik):
        # A function to catch singularity although doesn't work robustly
        print("singularity")
        current_j_states = ik.get_initial_pose()
        perbutration = np.zeros(7)
        perbutration[3] = -0.0001
        print(self.goal_joint_states())
        plan = np.linspace(current_j_states, current_j_states + perbutration, 10000)
        self.publish(plan)
        rospy.logwarn("**SINGULARITY OR OUT OF BOUNDS**")
        rospy.logwarn(f"On j_state {current_j_states}")
        rospy.logwarn(f"Perbutrated to {current_j_states + perbutration}")
        self. wait_for_execution()
        print(self.goal_joint_states())


if __name__ == "__main__":
    planner = TrajectoryPlanner('simple_planner_node')

    while not rospy.is_shutdown():
        planner.idle()
