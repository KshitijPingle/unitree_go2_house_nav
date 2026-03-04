import rclpy
from rclpy.node import Node
from go2_inter.srv import Cruising
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json


class GO2CruisingService(Node):
    def __init__(self):
        super().__init__('go2_cruising_service')
        self.get_logger().info('GO2 Cruising Service initialized')

        self.service = self.create_service(Cruising, 'cruising', self.cruising_callback)
        self.point = Point()

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.declare_parameter("x",0.1)
        self.declare_parameter("y",0.0)
        self.declare_parameter("z",0.0)

        # speed publisher
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        self.req_publisher = self.create_publisher(
            Request, '/api/sport/request', 10)
        self.timer = self.create_timer(0.1, self.on_timer)  

    def on_timer(self):
        req_msg = Request()
        req_msg.header.identity.api_id = self.api_id  # Set speed level to 1 (slow)
        js = {
            "x": self.get_parameter("x").get_parameter_value().double_value,
            "y": self.get_parameter("y").get_parameter_value().double_value,
            "z": self.get_parameter("z").get_parameter_value().double_value,
        }
        req_msg.parameter = json.dumps(js)
        self.req_publisher.publish(req_msg)  


    def odom_callback(self, odom:Odometry):
        self.point = odom.pose.pose.position

    def cruising_callback(self, request:Cruising.Request, response:Cruising.Response):
        flag = request.flag

        if flag == 0:
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
            self.get_logger().info('Over cruising')
        else:
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]
            self.get_logger().info('Start cruising')
        response.point = self.point
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GO2CruisingService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()