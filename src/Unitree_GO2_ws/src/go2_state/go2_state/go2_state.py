import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class GO2StateNode(Node):
    def __init__(self):
        super().__init__('go2_state')
        self.is_first = True
        self.last_x = 0.0
        self.last_y = 0.0

        self.declare_parameter("distance_threshold", 0.5)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('GO2 State Node started')

    def odom_callback(self, odom: Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        if self.is_first:
            self.is_first = False
            self.last_x = x
            self.last_y = y
            self.get_logger().info(f'First position ({x:.2f}, {y:.2f})')
            return
        
        dis_x = abs(x - self.last_x)
        dis_y = abs(y - self.last_y)
        distance = (dis_x**2 + dis_y**2)**0.5

        if distance >= self.get_parameter("distance_threshold").value:
            self.get_logger().info(f'Robot position ({x:.2f}, {y:.2f})')
            self.last_x = x
            self.last_y = y



def main(args=None):
    rclpy.init(args=args)
    node = GO2StateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()