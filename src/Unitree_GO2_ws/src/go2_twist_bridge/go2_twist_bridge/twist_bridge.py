import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from geometry_msgs.msg import Twist
from .sport_model import ROBOT_SPORT_API_IDS
import json

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        self.get_logger().info('GO2 Twist Bridge Node started')
        self.request_pub = self.create_publisher(
            # Assuming Request message is defined somewhere
            Request,
            '/api/sport/request',
            10
        )
        self.twist_sub = self.create_subscription(
            # Assuming Twist message is defined somewhere
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

    def twist_callback(self, twist: Twist):
        request = Request()
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        if x != 0.0 or y != 0.0 or z != 0.0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            js = {'x': x, 'y': y, 'z': z}
            request.parameter = json.dumps(js)
        request.header.identity.api_id = api_id
        self.request_pub.publish(request)

def main(args=None):
    rclpy.init(args=args)
    twist_bridge = TwistBridge()
    rclpy.spin(twist_bridge)
    twist_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()