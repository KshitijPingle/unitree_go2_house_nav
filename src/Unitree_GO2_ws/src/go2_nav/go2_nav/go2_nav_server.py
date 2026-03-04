import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, CancelResponse, ServerGoalHandle
from go2_inter.action import Nav
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from unitree_api.msg import Request
import json
import time
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion

# Assuming this is correctly imported from your sport_model.py
from .sport_model import ROBOT_SPORT_API_IDS


class GO2NavServer(Node):
    def __init__(self):
        super().__init__('go2_nav_server')
        self.get_logger().info('GO2 Navigation Server started')

        # State variables
        self.current_pos = Point(x=0.0, y=0.0, z=0.0)
        self.current_yaw = 0.0  # radians
        self.goal = None
        self.active_goal_handle = None
        self.api_id = ROBOT_SPORT_API_IDS.get("BALANCESTAND", 1)  # fallback

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Publisher for sport mode commands
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # Timer for sending velocity commands (~10 Hz)
        self.timer = self.create_timer(0.1, self.control_timer_callback)

        # Action server
        self.action_server = ActionServer(
            self,
            Nav,
            'go2_nav_action',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Server ready. Waiting for goals...')

    def odom_callback(self, msg: Odometry):
        self.current_pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def control_timer_callback(self):
        self.get_logger().info("Control timer tick - ALIVE!", throttle_duration_sec=1.0)
        if self.api_id not in [ROBOT_SPORT_API_IDS.get("MOVE", -1), ROBOT_SPORT_API_IDS.get("STOPMOVE", -1)]:

            return

        req = Request()
        req.header.identity.api_id = self.api_id

        if self.api_id == ROBOT_SPORT_API_IDS.get("STOPMOVE", -1):
            js = {"x": 0.0, "y": 0.0, "z": 0.0}
        elif self.goal is not None:
            # Compute errors
            dx = self.goal.x - self.current_pos.x
            dy = self.goal.y - self.current_pos.y
            distance = math.hypot(dx, dy)

            target_yaw = math.atan2(dy, dx)
            yaw_error = target_yaw - self.current_yaw
            # Normalize to [-π, π]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            vx = 0.0
            vy = 0.0
            vyaw = 0.0

            if abs(yaw_error) > math.radians(10):  # Turn first if off by >10°
                vyaw = 1.5 * yaw_error  # proportional, max ~1.5 rad/s
                vyaw = max(min(vyaw, 1.8), -1.8)
            else:
                speed = min(0.45, distance * 2.0)  # max 0.45 m/s, proportional near goal
                vx = speed * math.cos(yaw_error)   # small correction if still slightly off
                vy = speed * math.sin(yaw_error)

            js = {"x": vx, "y": vy, "z": vyaw}
            if distance < 0.12:
                self.api_id = ROBOT_SPORT_API_IDS.get("STOPMOVE", -1)
                js = {"x": 0.0, "y": 0.0, "z": 0.0}
                self.goal = None
                if self.active_goal_handle is not None:
                    self.active_goal_handle.succeed()
                    self.active_goal_handle = None
                    self.get_logger().info('Goal reached automatically (distance < 0.12 m)')
        else:
            js = {"x": 0.0, "y": 0.0, "z": 0.0}

        req.parameter = json.dumps(js)
        self.req_pub.publish(req)

    def goal_callback(self, goal_request: Nav.Goal):
        if self.active_goal_handle is not None:
            self.get_logger().warn('Rejecting new goal: another goal is active')
            return GoalResponse.REJECT

        if abs(goal_request.dx) < 0.01 and abs(goal_request.dy) < 0.01:
            self.get_logger().warn('Rejecting zero or tiny goal')
            return GoalResponse.REJECT

        self.goal = Point()
        self.goal.x = self.current_pos.x + goal_request.dx * math.cos(self.current_yaw) \
                      - goal_request.dy * math.sin(self.current_yaw)
        self.goal.y = self.current_pos.y + goal_request.dx * math.sin(self.current_yaw) \
                      + goal_request.dy * math.cos(self.current_yaw)
        self.goal.z = 0.0

        self.api_id = ROBOT_SPORT_API_IDS.get("MOVE", -1)
        self.active_goal_handle = None  # will be set in execute_callback

        self.get_logger().info(
            f'Accepted goal: relative ({goal_request.dx:.2f}, {goal_request.dy:.2f}) m → '
            f'absolute target ({self.goal.x:.2f}, {self.goal.y:.2f})'
        )
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.active_goal_handle = goal_handle
        feedback = Nav.Feedback()

        start_time = time.time()
        timeout_sec = 60.0

        self.get_logger().info('Executing navigation goal')

        while rclpy.ok():
            # Check if this is still our active goal (atomic check + early exit)
            if self.active_goal_handle is not goal_handle:
                self.get_logger().debug('Goal handle no longer matches active goal — exiting execute_callback')
                break

            if self.goal is None:
                self.get_logger().debug('No goal set anymore — exiting')
                break

            distance = math.hypot(
                self.goal.x - self.current_pos.x,
                self.goal.y - self.current_pos.y
            )
            feedback.distance = distance
            goal_handle.publish_feedback(feedback)

            if distance < 0.12:
                goal_handle.succeed()
                self.get_logger().info('Goal succeeded (distance < 0.12 m)')
                self.api_id = ROBOT_SPORT_API_IDS.get("STOPMOVE", -1)
                self.goal = None
                self.active_goal_handle = None
                break

            if time.time() - start_time > timeout_sec:
                goal_handle.abort()
                self.get_logger().error(f'Goal aborted: timeout after {timeout_sec} s')
                self.api_id = ROBOT_SPORT_API_IDS.get("STOPMOVE", -1)
                self.goal = None
                self.active_goal_handle = None
                break

            time.sleep(0.25)

        # Clean up if we exited loop without setting active_goal_handle to None
        if self.active_goal_handle is goal_handle:
            self.active_goal_handle = None

        result = Nav.Result()
        result.point = self.current_pos
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled by client')
        self.api_id = ROBOT_SPORT_API_IDS.get("STOPMOVE", -1)
        self.goal = None
        self.active_goal_handle = None
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = GO2NavServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()