#!/usr/bin/env python3
# by cenkt

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from simple_node import CommunicativeNode
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from math import pi, cos, sin, radians


class Robot(CommunicativeNode):
    def __init__(self):
        super(Robot, self).__init__("simple_robot_node")
        self.t_matrices = None
        self.theta = np.zeros(7)
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0])
        self.alpha = np.array([0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2, 0])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107])
        self.joints_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.joints_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.vel_max = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.acc_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20])
        self.jerk_max = np.array([7500, 3750, 5000, 6250, 7500, 10000, 10000])
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def callback(self, msg):
        self.theta = np.asarray(msg.position, dtype=np.float64)
        self.calculate_tf()

    def calculate_tf(self, q=None):
        if q is None:
            q = self.theta
        # 45 degrees is to match the x_e with the guiding (handle) part of the robot,
        # which is a good cue for camera rotation while working on simulation
        q = np.append(q, radians(45))
        t_matrices = np.zeros((11, 4, 4))
        t_matrices[0] = np.eye(4, 4)
        for i in range(len(q)):  # to start from 1, 0 is world frame
            t_matrices[i + 1] = t_matrices[i] @ self.tf_mat_from_dh(self.alpha[i], self.a[i], self.d[i], q[i])
        t_matrices[9] = t_matrices[8] @ self.tf_mat_from_dh(-np.pi / 2, 0, 0.057, 0)
        t_matrices[10] = t_matrices[8] @ self.tf_mat_from_dh(0, 0, 0.177, 0)
        self.t_matrices = t_matrices

    def check_limits(self, v, a, j, gas):
        cond_v = np.all(abs(v) < self.vel_max * gas)
        cond_a = np.all(abs(a) < self.acc_max * gas)
        cond_j = np.all(abs(j) < self.jerk_max * gas)
        return cond_v & cond_a & cond_j

    def check_joints(self, q):
        return (self.joints_min < q) & (q < self.joints_max)

    def get_current_pose(self, tf_index=8):
        self.calculate_tf(self.get_joint_states())
        return self.t_matrices[tf_index]

    @staticmethod
    def tf_mat_from_dh(alpha, a, d, angle):
        tf_matrix = np.array([[cos(angle), -sin(angle), 0, a],
                              [sin(angle) * cos(alpha), cos(angle) * cos(alpha), -sin(alpha), -d * sin(alpha)],
                              [sin(angle) * sin(alpha), cos(angle) * sin(alpha), cos(alpha), d * cos(alpha)],
                              [0, 0, 0, 1]])
        return tf_matrix

    def broadcast_tf(self, t_matrix, name):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = name
        t.transform.translation.x = t_matrix[0, -1]
        t.transform.translation.y = t_matrix[1, -1]
        t.transform.translation.z = t_matrix[2, -1]
        q = tf_conversions.transformations.quaternion_from_matrix(t_matrix)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def broadcast_all_tfs(self):
        self.broadcast_tf(self.t_matrices[8], 'tf_flange')
        self.broadcast_tf(self.t_matrices[9], 'tf_camera')
        self.broadcast_tf(self.t_matrices[10], 'tf_needle')


if __name__ == "__main__":
    robot = Robot()
    robot.init_subscriber(robot.j_states_topic, JointState, robot.callback)

    while robot.t_matrices is None:
        rospy.sleep(0.1)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        robot.broadcast_all_tfs()
        r.sleep()
