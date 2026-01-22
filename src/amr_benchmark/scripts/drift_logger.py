#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import csv
import time

class DriftLogger(Node):
    def __init__(self):
        super().__init__('drift_logger')

        self.gt_pose = None
        self.robot_name = "amr_robot"

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gt_cb, 10)

        self.odom_data = None
        self.ekf_data = None

        self.file = open('drift_log.csv', 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'gt_x', 'gt_y',
            'odom_x', 'odom_y',
            'ekf_x', 'ekf_y'
        ])

        self.get_logger().info("Drift logger started")

    def gt_cb(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            self.gt_pose = msg.pose[idx]

    def odom_cb(self, msg):
        self.odom_data = msg.pose.pose
        self.log_if_ready()

    def ekf_cb(self, msg):
        self.ekf_data = msg.pose.pose
        self.log_if_ready()

    def log_if_ready(self):
        if self.gt_pose and self.odom_data and self.ekf_data:
            t = time.time()
            self.writer.writerow([
                t,
                self.gt_pose.position.x,
                self.gt_pose.position.y,
                self.odom_data.position.x,
                self.odom_data.position.y,
                self.ekf_data.position.x,
                self.ekf_data.position.y
            ])
            self.file.flush()

def main():
    rclpy.init()
    node = DriftLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
