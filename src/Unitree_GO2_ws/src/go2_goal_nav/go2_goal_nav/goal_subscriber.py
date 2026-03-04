import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from unitree_api.msg import Request
import json
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from .sport_model import ROBOT_SPORT_API_IDS   # 保持你的导入


class GO2GoalPoseNavigator(Node):
    def __init__(self):
        super().__init__('go2_goal_pose_navigator')
        self.get_logger().info('GO2 Goal Pose Navigator started (using /goal_pose from rviz)')

        # 状态变量
        self.current_pose = PoseStamped()          # 当前位姿 (map 帧)
        self.current_yaw = 0.0
        self.goal_pose = None                      # 目标位姿 (收到后设置)
        self.goal_received_time = 0.0
        self.timeout_sec = 120.0                   # 超时自动放弃（秒）

        
        
        # 订阅 odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # 订阅 rviz2 的 goal_pose（最关键）
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 发布 sport request
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # 控制定时器 ~10Hz
        self.timer = self.create_timer(
            0.2, 
            self.control_timer_callback,
        )

        self.get_logger().info('Waiting for /goal_pose clicks in rviz2...')

    def odom_callback(self, msg: Odometry):
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        q = msg.pose.pose.orientation
        (_, _, self.current_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "odom":
            self.get_logger().warn(f"Ignoring goal in frame '{msg.header.frame_id}' (expected 'map')")
            return

        self.goal_pose = msg
        self.goal_received_time = self.get_clock().now().nanoseconds / 1e9
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gyaw = self._get_yaw(msg)
        self.get_logger().info(
            f"New goal from rviz: ({gx:.2f}, {gy:.2f}) yaw={math.degrees(gyaw):.1f}°"
        )

    def _get_yaw(self, pose: PoseStamped):
        q = pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def control_timer_callback(self):
        if self.goal_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().debug(f"Time since goal received: {now - self.goal_received_time:.1f}s")
        if now - self.goal_received_time > self.timeout_sec:
            self.get_logger().warn("Goal timeout → stopping and clearing goal")
            self.send_stop()
            self.goal_pose = None
            return

        # 当前
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cyaw = self.current_yaw

        # 目标
        tx = self.goal_pose.pose.position.x
        ty = self.goal_pose.pose.position.y
        tyaw = self._get_yaw(self.goal_pose)

        dx = tx - cx
        dy = ty - cy
        distance = math.hypot(dx, dy)

        # 朝向目标方向的偏航误差
        heading_error = math.atan2(dy, dx) - cyaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # [-pi, pi]

        # 最终朝向误差（可选，如果你希望到达后也对准目标 yaw）
        final_yaw_error = tyaw - cyaw
        final_yaw_error = math.atan2(math.sin(final_yaw_error), math.cos(final_yaw_error))

        if distance > 0.15:  # 到达阈值，可调 0.12~0.20
            vx = 0.0
            vy = 0.0
            vyaw = 0.0

            if abs(heading_error) > math.radians(12):  # 先转弯（阈值可调）
                vyaw = 1.8 * heading_error
                vyaw = max(min(vyaw, 2.0), -2.0)
            else:
                # 前进 + 小幅修正
                speed = min(0.50, distance * 1.8)  # 最大速度 0.5 m/s，可调
                vx = speed * math.cos(heading_error)
                vy = speed * math.sin(heading_error)
                vyaw = 0.8 * heading_error + 0.5 * final_yaw_error  # 轻微融入最终朝向

            self.send_velocity(vx, vy, vyaw)
            self.get_logger().debug(
                f"dist={distance:.2f}m  heading_err={math.degrees(heading_error):.1f}° → vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}"
            )
        else:
            # 已接近 → 微调最终朝向或直接停
            if abs(final_yaw_error) > math.radians(8):
                self.send_velocity(0.0, 0.0, 1.2 * final_yaw_error)
            else:
                self.send_stop()
                self.get_logger().info(f"Goal reached! dist={distance:.2f}m yaw_err={math.degrees(final_yaw_error):.1f}°")
                self.goal_pose = None

    def send_velocity(self, vx: float, vy: float, vyaw: float):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_IDS.get("MOVE",1008)  # 请确认实际值！
        param = {"x": float(vx), "y": float(vy), "z": float(vyaw)}
        req.parameter = json.dumps(param)
        self.get_logger().debug(f"Publishing velocity command: {param}",throttle_duration_sec=1.0)
        self.req_pub.publish(req)

    def send_stop(self):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_IDS.get("STOPMOVE", 1003)  # 请确认实际值！
        req.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
        self.req_pub.publish(req)

def main(args=None):
    rclpy.init(args=args)
    node = GO2GoalPoseNavigator()

    # 用 MultiThreadedExecutor（推荐）
    executor = MultiThreadedExecutor()  # 或参数指定线程数，如 MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()