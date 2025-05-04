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
        self.camera_subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.camera_callback, 1)

        self.latest_cmd = None # keep track of last command
        self.odom_speed = None

        self.color_thresh = {
            'red': [{'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)}],
            'yellow': [{'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)}],
            'green': [{'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)}],
            }

        self.color_idx = {0: 'red', 1: 'yellow', 2: 'green'}

    # def safe_distance_function(self, speed):
    #     if speed > 2.5:
    #         return 1.4 + self.GOAL_DISTANCE
    #     return .767 * speed - .437 + self.GOAL_DISTANCE

    def safe_distance_function(self, speed):
        return (0.47 * speed - 0.3) + self.GOAL_DISTANCE

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().info(f"Failed to convert image {e}")

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        areas = {'red': 0, 'yellow': 0, 'green': 0}

        for color, ranges in self.color_thresh.items():
            mask = None
            for rng in ranges:
                m = cv2.inRange(hsv_img, rng['lower'], rng['upper'])
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            for cnt in contours:
                max_area = max(max_area, cv2.contourArea(cnt))
            areas[color] = max_area

        signal = max(areas, key=areas.get)
        should_stop = (signal == 'red')

        mask = None
        for rng in self.color_thresh[signal]:
            m = cv2.inRange(hsv_img, rng['lower'], rng['upper'])
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)
        
        # publish stop
        # stop_msg = Bool()
        # stop_msg.data = should_stop
        # self.command_pub.publish(stop_msg)
        
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = 0.0
        self.drive_pub.publish(drive_cmd)
        self.get_logger().info(f'Detected {signal}, should_stop={should_stop}')
        self.get_logger().info(f'{areas}')

    def scan_callback(self, scan):
        if self.latest_cmd is None or self.odom_speed is None:
            return
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        steer_angle = self.drive_cmd.drive.steering_angle

        lower_bound = np.deg2rad(steer_angle-self.BOUND_ANGLE)
        upper_bound = np.deg2rad(steer_angle+self.BOUND_ANGLE)
        left_bound_min = np.deg2rad(-self.BOUND_ANGLE+self.left_right_angle)
        left_bound_max = np.deg2rad(self.BOUND_ANGLE+self.left_right_angle)
        right_bound_min = np.deg2rad(-self.BOUND_ANGLE-self.left_right_angle)
        right_bound_max = np.deg2rad(self.BOUND_ANGLE-self.left_right_angle)
        indices = np.where((angles >= lower_bound) & (angles <= upper_bound))
        left_indices = np.where((angles >= left_bound_min) & (angles <= left_bound_max))
        right_indices = np.where((angles >= right_bound_min) & (angles <= right_bound_max))

        filtered_ranges = ranges[indices]
        left_filtered_ranges = ranges[left_indices]
        right_filtered_ranges = ranges[right_indices]
        min_distance = np.min(filtered_ranges)
        min_left_distance = np.min(left_filtered_ranges)
        min_right_distance = np.min(right_filtered_ranges)
        self.get_logger().info(f"Min distance: {min_distance}")

        speed = self.odom_speed
        safe_distance = self.safe_distance_function(speed)
        if safe_distance < self.GOAL_DISTANCE:
            safe_distance = self.GOAL_DISTANCE
        
        if min_distance < safe_distance:
            if steer_angle >= 0:
                if min_left_distance > safe_distance:
                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.speed = self.latest_cmd.drive.speed
                    drive_msg.drive.steer_angle = self.left_right_angle
                    self.safety_pub.publish(drive_msg)
                    return
                elif min_right_distance > safe_distance:
                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.speed = self.latest_cmd.drive.speed
                    drive_msg.drive.steer_angle = -self.left_right_angle
                    self.safety_pub.publish(drive_msg)
                    return
            elif steer_angle < 0:
                if min_right_distance > safe_distance:
                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.speed = self.latest_cmd.drive.speed
                    drive_msg.drive.steer_angle = -self.left_right_angle
                    self.safety_pub.publish(drive_msg)
                    return
                elif min_left_distance > safe_distance:
                    drive_msg = AckermannDriveStamped()
                    drive_msg.drive.speed = self.latest_cmd.drive.speed
                    drive_msg.drive.steer_angle = self.left_right_angle
                    self.safety_pub.publish(drive_msg)
                    return
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