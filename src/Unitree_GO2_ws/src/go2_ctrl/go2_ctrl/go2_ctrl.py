#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json

class GO2Control(Node):
    def __init__(self):
        super().__init__('go2_control')
        # parameterize the control commands
        self.declare_parameter("sport_api_id", ROBOT_SPORT_API_IDS['BALANCESTAND'])
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)

        
        self.get_logger().info('GO2 Control Node started')
        # create a publisher for the GO2 control commands
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        # create a timer to send the control commands periodically
        self.control_timer = self.create_timer(0.1, self.send_control_commands)


    def send_control_commands(self):
        req = Request()

        id = self.get_parameter('sport_api_id').get_parameter_value().integer_value
        req.header.identity.api_id = id

        if id == ROBOT_SPORT_API_IDS['MOVE']:
            js = json.dumps({
                    'x': self.get_parameter('x').get_parameter_value().double_value,
                    'y': self.get_parameter('y').get_parameter_value().double_value,
                    'z': self.get_parameter('z').get_parameter_value().double_value,
            })
            req.parameter = js
        # Implement the logic to send control commands here
            self.req_pub.publish(req)  # Replace with the actual control command


def main(args=None):
    rclpy.init(args=args)
    node = GO2Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()