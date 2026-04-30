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

        self.declare_parameter('front_angle_deg', 45.0)
        self.declare_parameter('side_angle_deg', 90.0)

        self.declare_parameter('stop_distance', 0.70)
        self.declare_parameter('slow_distance', 1.0)

        self.declare_parameter('resume_distance', 0.90)
        self.declare_parameter('turn_speed', 0.80)

        self.declare_parameter('max_linear_scale_when_slow', 0.25)

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

        self.resume_distance = float(self.get_parameter('resume_distance').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        self.max_linear_scale_when_slow = float(self.get_parameter('max_linear_scale_when_slow').value)
        self.max_angular_scale_when_blocked = float(self.get_parameter('max_angular_scale_when_blocked').value)

        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.max_valid_range = float(self.get_parameter('max_valid_range').value)

        self.latest_cmd = Twist()
        self.latest_scan = None

        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.left_clearance = math.inf
        self.right_clearance = math.inf


        self.turning = False
        self.turn_direction = 1.0

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
                elif front_half < angle <= side_half:
                    left_ranges.append(r)
                elif -side_half <= angle < -front_half:
                    right_ranges.append(r)
            angle += msg.angle_increment

        self.front_min = min(front_ranges) if front_ranges else math.inf
        self.left_min = min(left_ranges) if left_ranges else math.inf
        self.right_min = min(right_ranges) if right_ranges else math.inf

        self.left_clearance = self.median_or_inf(left_ranges)
        self.right_clearance = self.median_or_inf(right_ranges)

    def is_valid_range(self, r: float) -> bool:
        return math.isfinite(r) and self.min_valid_range <= r <= self.max_valid_range
    
    def median_or_inf(self, values: List[float]) -> float:
        if not values:
            return math.inf
        vals = sorted(values)
        n = len(vals)
        if n % 2 == 1:
            return vals[n // 2]
        return 0.5 * (vals[n // 2 - 1] + vals[n // 2])

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

        if self.turning:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0

            if self.front_min > self.resume_distance:
                safe_cmd.angular.z = 0.0
                self.turning = False
                status = 'CLEAR_AFTER_TURN'
            else:
                safe_cmd.angular.z = self.turn_direction * self.turn_speed
                status = 'TURNING_LEFT' if self.turn_direction > 0 else 'TURNING_RIGHT'

        elif self.front_min < self.stop_distance:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            safe_cmd.angular.z = 0.0

            if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
                self.turn_direction = 1.0 if self.left_clearance > self.right_clearance else -1.0
            elif math.isfinite(self.left_clearance):
                self.turn_direction = 1.0
            elif math.isfinite(self.right_clearance):
                self.turn_direction = -1.0
            else:
                self.turn_direction = 1.0

            self.turning = True
            status = 'STOP_FRONT_OBSTACLE'

        elif moving_forward and self.front_min < self.slow_distance:
            scale = max(
                self.max_linear_scale_when_slow,
                (self.front_min - self.stop_distance) / max(self.slow_distance - self.stop_distance, 1e-6)
            )
            scale = min(max(scale, self.max_linear_scale_when_slow), 1.0)
            safe_cmd.linear.x *= scale
            status = f'SLOW_FRONT_OBSTACLE scale={scale:.2f}'
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
