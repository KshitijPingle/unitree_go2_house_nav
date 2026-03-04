import rclpy
from rclpy.node import Node
import sys
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from go2_inter.action import Nav

class GO2NavClient(Node):
    def __init__(self):
        super().__init__('go2_nav_client')
        self.get_logger().info('GO2 Navigation Client started')
        self.client = ActionClient(self, Nav, 'go2_nav_action')

    def connect_server(self):
        self.get_logger().info('Connecting to GO2 Navigation Server...')
        while not self.client.wait_for_server(1.0):
            self.get_logger().info('Waiting for server...')
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the server. Exiting.')
                return False
        return True
    
    def send_goal(self, dx, dy):
        goal_msg = Nav.Goal()
        goal_msg.dx = dx
        goal_msg.dy = dy

        future = self.client.send_goal_async(goal_msg,self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Current Distance to Goal: {feedback.distance}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('GO2 Navigation Goal rejected')
            return

        self.get_logger().info('GO2 Navigation Goal accepted')


def main(args=None):
    if len(sys.argv) != 3:
        get_logger('GO2NavClient').error('Only float goal distance argument is accepted.')
        return
    rclpy.init(args=args)

    node = GO2NavClient()
    flag = node.connect_server()
    if not flag:
        get_logger('GO2NavClient').error('Failed to connect to GO2 Navigation Server.')
        return
    get_logger('GO2NavClient').info('Successfully connected to GO2 Navigation Server.')
    
    # Send goal
    node.send_goal(float(sys.argv[1]), float(sys.argv[2]))
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()