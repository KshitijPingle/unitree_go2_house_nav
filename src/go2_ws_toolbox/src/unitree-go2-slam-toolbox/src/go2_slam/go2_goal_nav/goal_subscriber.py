import math
import rclpy
import json
import os
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import LoadMap, SaveMap
from geometry_msgs.msg import PoseStamped, Point
from unitree_api.msg import Request
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from datetime import datetime
from std_msgs.msg import String
from enum import Enum
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

class MapManagerNode(Node):
    def __init__(self):
        super().__init__("map_manager")

        # Config
        self.declare_parameter("maps_root", "/tmp/maps")
        self.declare_parameter("map_name", "house_map")
        self.declare_parameter("map_topic", "/map")

        self.maps_root = self.get_parameter("maps_root").value
        self.map_name = self.get_parameter("map_name").value
        self.map_topic = self.get_parameter("map_topic").value

        os.makedirs(self.maps_root, exist_ok=True)

        # Clients to existing map services
        self.load_client = self.create_client(LoadMap, "/map_server/load_map")
        self.save_client = self.create_client(SaveMap, "/map_saver/save_map")

        # Your public API (simple triggers)
        self.create_service(Trigger, "/map_manager/load", self.handle_load)
        self.create_service(Trigger, "/map_manager/save", self.handle_save)
        self.create_service(Trigger, "/map_manager/update", self.handle_update)

        self.get_logger().info("MapManagerNode ready: /map_manager/load|save|update")

    def _next_version_dir(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.maps_root, self.map_name, f"v_{ts}")

    def handle_load(self, request, response):
        target_yaml = os.path.join(self.maps_root, self.map_name, "current", "map.yaml")

        if not os.path.exists(target_yaml):
            response.success = False
            response.message = f"Map not found: {target_yaml}"
            return response

        if not self.load_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "load_map service unavailable"
            return response

        req = LoadMap.Request()
        req.map_url = target_yaml
        future = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            response.success = False
            response.message = "load_map call failed"
            return response

        response.success = True
        response.message = f"Loaded map: {target_yaml}"
        return response

    def handle_save(self, request, response):
        version_dir = self._next_version_dir()
        os.makedirs(version_dir, exist_ok=True)
        map_url = os.path.join(version_dir, "map")

        if not self.save_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "save_map service unavailable"
            return response

        req = SaveMap.Request()
        req.map_topic = self.map_topic
        req.map_url = map_url
        req.image_format = "pgm"
        req.map_mode = "trinary"
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65

        future = self.save_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            response.success = False
            response.message = "save_map call failed"
            return response

        # Optional: update "current" pointer/symlink in your own logic
        response.success = True
        response.message = f"Saved map version: {version_dir}"
        return response

    def handle_update(self, request, response):
        # Simple policy: update == save as new version.
        # If you use slam_toolbox merge/serialize, call that first, then save.
        return self.handle_save(request, response)


class NavState(Enum):
    IDLE = "IDLE"
    FOLLOW_GOAL = "FOLLOW_GOAL"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    GOAL_REACHED = "GOAL_REACHED"


class DistanceAwareNavigator(Node):
    def __init__(self):
        super().__init__("distance_aware_navigator")

        self.goal = None
        self.cx = 0.0
        self.cy = 0.0
        self.cyaw = 0.0

        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")

        self.state = NavState.IDLE
        self.local_wp = None  # (x, y) in odom frame

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        self.req_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.status_pub = self.create_publisher(String, "/nav_status", 10)

        self.timer = self.create_timer(0.1, self.control_cb)

    def odom_cb(self, msg: Odometry):
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.cyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def goal_cb(self, msg: PoseStamped):
        if msg.header.frame_id not in ("odom", ""):
            self.get_logger().warn(f"Expected goal in odom frame, got {msg.header.frame_id}")
            return
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.state = NavState.FOLLOW_GOAL
        self.get_logger().info(f"New goal: {self.goal}")

    def scan_cb(self, msg: LaserScan):
        def min_in_window(center_deg, width_deg):
            c = math.radians(center_deg)
            w = math.radians(width_deg)
            best = float("inf")
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r):
                    continue
                a = msg.angle_min + i * msg.angle_increment
                d = math.atan2(math.sin(a - c), math.cos(a - c))
                if abs(d) <= w * 0.5:
                    best = min(best, r)
            return best

        self.front_min = min_in_window(0, 40)
        self.left_min = min_in_window(60, 40)
        self.right_min = min_in_window(-60, 40)

    def control_cb(self):
        if self.goal is None:
            self.send_stop()
            self.publish_status()
            return

        gx, gy = self.goal
        goal_dist = math.hypot(gx - self.cx, gy - self.cy)

        if goal_dist < 0.25:
            self.state = NavState.GOAL_REACHED
            self.send_stop()
            self.publish_status()
            return

        if self.front_min < 0.45:
            self.state = NavState.AVOID_OBSTACLE
            self.local_wp = self.generate_avoidance_waypoint(gx, gy)
        else:
            self.state = NavState.FOLLOW_GOAL
            self.local_wp = self.generate_goal_waypoint(gx, gy)

        self.follow_waypoint(self.local_wp)
        self.publish_status()

    def generate_goal_waypoint(self, gx, gy):
        heading = math.atan2(gy - self.cy, gx - self.cx)
        step = min(0.8, math.hypot(gx - self.cx, gy - self.cy))
        return (self.cx + step * math.cos(heading), self.cy + step * math.sin(heading))

    def generate_avoidance_waypoint(self, gx, gy):
        base_heading = math.atan2(gy - self.cy, gx - self.cx)
        turn = math.radians(45) if self.left_min > self.right_min else -math.radians(45)
        heading = base_heading + turn
        step = 0.5
        return (self.cx + step * math.cos(heading), self.cy + step * math.sin(heading))

    def follow_waypoint(self, wp):
        wx, wy = wp
        dx, dy = wx - self.cx, wy - self.cy
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - self.cyaw), math.cos(target_yaw - self.cyaw))

        if abs(yaw_err) > math.radians(15):
            vx, vy, wz = 0.0, 0.0, max(min(1.5 * yaw_err, 1.2), -1.2)
        else:
            speed = min(0.4, dist * 1.2)
            vx = speed * math.cos(yaw_err)
            vy = speed * math.sin(yaw_err)
            wz = max(min(0.8 * yaw_err, 0.8), -0.8)

        self.send_velocity(vx, vy, wz)

    def send_velocity(self, vx, vy, wz):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_IDS.get("MOVE", 1008)
        req.parameter = json.dumps({"x": float(vx), "y": float(vy), "z": float(wz)})
        self.req_pub.publish(req)

    def send_stop(self):
        req = Request()
        req.header.identity.api_id = ROBOT_SPORT_API_IDS.get("STOPMOVE", 1003)
        req.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
        self.req_pub.publish(req)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "state": self.state.value,
            "front_min": self.front_min,
            "left_min": self.left_min,
            "right_min": self.right_min,
            "waypoint": self.local_wp
        })
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GO2GoalPoseNavigator()
    map_node = MapManagerNode()
    distance_node = DistanceAwareNavigator()

    # 用 MultiThreadedExecutor（推荐）
    executor = MultiThreadedExecutor()  # 或参数指定线程数，如 MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(map_node)
    executor.add_node(distance_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        map_node.destroy_node()
        distance_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()