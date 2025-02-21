#!/usr/bin/env python3

import copy
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HardwareStatusPublisherNode(Node):

    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.hw_status_publisher_ = self.create_publisher(Float64MultiArray, "ur_left_joint_group_pos_controller/commands", 10)
        #self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.ur_type = "ur_left"
        # self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Robot status publisher has been started.")

    def timer_callback(self):  # Add indentation here
        msg = Float64MultiArray()
        print("aaagaa")
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        traj = [[2.0, 1.0, 2.0, 1.0, 2.0, 1.0], [1.2, 2.0, 1.1, 1.5, 1.6, 1.5]]  # Assign the list of positions to the message
        for i in traj:
            joint_pose = Float64MultiArray()
            joint_pose.data = copy.deepcopy(list(i))
            self.hw_status_publisher_.publish(joint_pose)
            print("aaagaa")
            time.sleep(0.002)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode()
    node.timer_callback()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
