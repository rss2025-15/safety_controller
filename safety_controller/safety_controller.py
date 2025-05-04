#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
import time

from std_msgs.msg import Header
from wall_follower.visualization_tools import VisualizationTools

class SafetyController(Node):

    def __init__(self):
        super().__init__('safety_controller')
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("safety_topic", "default")
        self.declare_parameter("goal_distance", 1.)
        self.declare_parameter("bound_angle", 5.)
        self.declare_parameter("left_right_angle", 15.0)

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.GOAL_DISTANCE= self.get_parameter('goal_distance').get_parameter_value().double_value
        self.BOUND_ANGLE = self.get_parameter('bound_angle').get_parameter_value().double_value
        self.left_right_angle = self.get_parameter('left_right_angle').get_parameter_value().double_value

        self.safety_pub = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)

        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_drive = self.create_subscription(AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/vesc/odom", self.odom_callback, 10)

        self.latest_cmd = None # keep track of last command
        self.odom_speed = None

    # def safe_distance_function(self, speed):
    #     if speed > 2.5:
    #         return 1.4 + self.GOAL_DISTANCE
    #     return .767 * speed - .437 + self.GOAL_DISTANCE

    def safe_distance_function(self, speed):
        return (0.47 * speed - 0.3) + self.GOAL_DISTANCE

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        lower_bound = np.deg2rad(-self.BOUND_ANGLE)
        upper_bound = np.deg2rad(self.BOUND_ANGLE)
        left_bound = np.deg2rad(-self.BOUND_ANGLE-self.left_right_angle)
        right_bound = np.deg2rad(self.BOUND_ANGLE+self.left_right_angle)
        indices = np.where((angles >= lower_bound) & (angles <= upper_bound))
        left_indices = np.where((angles >= left_bound) & (angles <= lower_bound))
        right_indices = np.where((angles >= upper_bound) & (angles <= right_bound))

        filtered_ranges = ranges[indices]
        left_filtered_ranges = ranges[left_indices]
        right_filtered_ranges = ranges[right_indices]
        min_distance = np.min(filtered_ranges)
        self.get_logger().info(f"Min distance: {min_distance}")

        if min_distance < self.GOAL_DISTANCE:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.safety_pub.publish(drive_msg)

        # if self.odom_speed:
        #     # self.get_logger().info(f"Receiving command")
        #     # speed = self.latest_cmd.drive.speed
        #     speed = self.odom_speed
        #     safe_distance = self.safe_distance_function(speed)
        #     if safe_distance < self.GOAL_DISTANCE:
        #         safe_distance = self.GOAL_DISTANCE

        #     if min_distance < safe_distance:
        #         # self.get_logger().info(f"STOPPING")
        #         drive_msg = AckermannDriveStamped()
        #         drive_msg.drive.speed = 0.0
        #         self.safety_pub.publish(drive_msg)
        #     # else:
        #     #     if self.latest_cmd:
        #     #     self.safety_pub.publish(self.latest_cmd)


    def drive_callback(self, drive_msg):
        self.latest_cmd = drive_msg
        # self.get_logger().info(f"latest command: {self.latest_cmd}")

    def odom_callback(self, odom_msg):
        self.odom_speed = odom_msg.twist.twist.linear.x
        # self.get_logger().info(f'GOT ODOM SPEED: {self.odom_speed}')

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
