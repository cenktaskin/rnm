#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState


class CommunicativeNode:
    def __init__(self, node_name=None):
        if node_name is not None:
            rospy.init_node(node_name)
        self.publisher = None
        self.subscriber = None
        self.execution_publisher = None
        sim = rospy.get_param("use_sim_time", False)
        self.goal_topic = "/goal_pose"
        self.execution_topic = "/execution_state"
        self.ir_topic = "/k4a/ir/image_raw"
        self.rgb_topic = "/k4a/rgb/image_raw"
        if not sim:
            self.execution_target = "hardware"
            self.command_topic = "/joint_position_example_controller/joint_command"
            self.j_states_topic = "/franka_state_controller/joint_states_desired"
        else:
            self.execution_target = "simulation"
            self.command_topic = "/joint_position_example_controller_sim/joint_command"
            self.j_states_topic = "/joint_states"

    def init_publisher(self, topic=None, msg_type=Float64MultiArray):
        if topic is None:
            topic = self.goal_topic
        self.publisher = rospy.Publisher(topic, msg_type, queue_size=10)

    def init_subscriber(self, topic, msg_type, callback):
        self.subscriber = rospy.Subscriber(topic, msg_type, callback)

    def init_execution_publisher(self):
        self.execution_publisher = rospy.Publisher(self.execution_topic, String, queue_size=10)

    def publish(self, data):
        self.publisher.publish(Float64MultiArray(data=np.array(data, dtype=np.float64).flatten()))

    def wait_for_execution(self, target_state=None):
        while True:
            msg = rospy.wait_for_message(self.execution_topic, String).data
            rospy.loginfo(f"Comm_Node: Message received '{msg}'")
            if msg == target_state or target_state is None:
                return msg

    def wait_for_connections(self):
        if self.publisher is None:
            rospy.logwarn(f"Comm Node: PUBLISHER IS NOT YET INITIALIZED")
        else:
            while self.publisher.get_num_connections() == 0:
                rospy.sleep(0.1)
            rospy.loginfo(f"Got {self.publisher.get_num_connections()} connection(s)")

    def get_joint_states(self):
        return np.array(rospy.wait_for_message(self.j_states_topic, JointState, rospy.Duration(10)).position)
