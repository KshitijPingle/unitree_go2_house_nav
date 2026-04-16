#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ObstacleAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__('go2_obstacle_avoidance')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_safe')
        self.declare_parameter('status_topic', '/obstacle_avoidance/status')

        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('side_angle_deg', 70.0)

        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('slow_distance', 0.80)
        self.declare_parameter('turn_slow_distance', 0.40)

        self.declare_parameter('max_linear_scale_when_slow', 0.25)
        self.declare_parameter('max_angular_scale_when_blocked', 0.40)

        self.declare_parameter('min_valid_range', 0.05)
        self.declare_parameter('max_valid_range', 8.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.side_angle_deg = float(self.get_parameter('side_angle_deg').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.turn_slow_distance = float(self.get_parameter('turn_slow_distance').value)

        self.max_linear_scale_when_slow = float(self.get_parameter('max_linear_scale_when_slow').value)
        self.max_angular_scale_when_blocked = float(self.get_parameter('max_angular_scale_when_blocked').value)

        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.max_valid_range = float(self.get_parameter('max_valid_range').value)

        self.latest_cmd = Twist()
        self.latest_scan = None

        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, self.input_cmd_topic, self.cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstacle avoidance node started')

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

        front_half = math.radians(self.front_angle_deg)
        side_half = math.radians(self.side_angle_deg)

        front_ranges: List[float] = []
        left_ranges: List[float] = []
        right_ranges: List[float] = []

        angle = msg.angle_min
        for r in msg.ranges:
            if self.is_valid_range(r):
                if -front_half <= angle <= front_half:
                    front_ranges.append(r)
                if 0.0 < angle <= side_half:
                    left_ranges.append(r)
                if -side_half <= angle < 0.0:
                    right_ranges.append(r)
            angle += msg.angle_increment

        self.front_min = min(front_ranges) if front_ranges else math.inf
        self.left_min = min(left_ranges) if left_ranges else math.inf
        self.right_min = min(right_ranges) if right_ranges else math.inf

    def is_valid_range(self, r: float) -> bool:
        return math.isfinite(r) and self.min_valid_range <= r <= self.max_valid_range

    def control_loop(self) -> None:
        safe_cmd = Twist()
        safe_cmd.linear.x = self.latest_cmd.linear.x
        safe_cmd.linear.y = self.latest_cmd.linear.y
        safe_cmd.linear.z = self.latest_cmd.linear.z
        safe_cmd.angular.x = self.latest_cmd.angular.x
        safe_cmd.angular.y = self.latest_cmd.angular.y
        safe_cmd.angular.z = self.latest_cmd.angular.z

        status = 'CLEAR'

        if self.latest_scan is None:
            self.cmd_pub.publish(safe_cmd)
            self.publish_status('NO_SCAN')
            return

        moving_forward = safe_cmd.linear.x > 0.01

        if moving_forward and self.front_min < self.stop_distance:
            safe_cmd.linear.x = 0.0
            status = 'STOP_FRONT_OBSTACLE'

            if self.left_min < self.right_min:
                safe_cmd.angular.z = min(safe_cmd.angular.z, 0.0)
                if abs(safe_cmd.angular.z) < 0.1:
                    safe_cmd.angular.z = -self.max_angular_scale_when_blocked
            else:
                safe_cmd.angular.z = max(safe_cmd.angular.z, 0.0)
                if abs(safe_cmd.angular.z) < 0.1:
                    safe_cmd.angular.z = self.max_angular_scale_when_blocked

        elif moving_forward and self.front_min < self.slow_distance:
            scale = max(
                self.max_linear_scale_when_slow,
                (self.front_min - self.stop_distance) / max(self.slow_distance - self.stop_distance, 1e-6)
            )
            scale = min(max(scale, self.max_linear_scale_when_slow), 1.0)
            safe_cmd.linear.x *= scale
            status = f'SLOW_FRONT_OBSTACLE scale={scale:.2f}'

        turning = abs(safe_cmd.angular.z) > 0.05
        if turning:
            if safe_cmd.angular.z > 0.0 and self.left_min < self.turn_slow_distance:
                safe_cmd.angular.z *= self.max_angular_scale_when_blocked
                if status == 'CLEAR':
                    status = 'SLOW_LEFT_TURN'
            elif safe_cmd.angular.z < 0.0 and self.right_min < self.turn_slow_distance:
                safe_cmd.angular.z *= self.max_angular_scale_when_blocked
                if status == 'CLEAR':
                    status = 'SLOW_RIGHT_TURN'

        self.cmd_pub.publish(safe_cmd)
        self.publish_status(f'{status} | front={self.front_min:.2f} left={self.left_min:.2f} right={self.right_min:.2f}')

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
