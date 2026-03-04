import rclpy
from rclpy.node import Node
import sys
from rclpy.logging import get_logger
from go2_inter.srv import Cruising
from rclpy.task import Future

class Go2CruisingClient(Node):
    def __init__(self):
        super().__init__('go2_cruising_client')
        self.get_logger().info('GO2 Cruising Client node started')
        self.clinet = self.create_client(Cruising, 'cruising')

    def connect_service(self):
        while not self.clinet.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return False
            self.get_logger().info(f'Service not available, waiting again...')
        return True
    def send_request(self, flag) -> Future:
        req = Cruising.Request()
        req.flag = flag
        return self.clinet.call_async(req)

def main(args=None):
    if len(sys.argv) != 2:
        get_logger("Usage: go2_cruising_client <flag>").error("flag: 1 to start cruising, 0 to stop cruising")
        return
    rclpy.init(args=args)
    client = Go2CruisingClient()
    flag = client.connect_service()
    if not flag:
        return
    future = client.send_request(int(sys.argv[1]))
    rclpy.spin_until_future_complete(client, future)
    if future.done():
        try:
            response = future.result()
            get_logger("go2_cruising_client").info('Robotic Dog Position(%.2f, %.2f)' % (response.point.x, response.point.y))
        except Exception as e:
            client.get_logger().error(f'Service call failed: {e}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()