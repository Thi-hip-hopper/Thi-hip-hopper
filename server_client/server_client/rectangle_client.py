import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import RectangleDimensions

class RectangleClient(Node):
    def __init__(self):
        super().__init__('rectangle_client')
        self.cli = self.create_client(RectangleDimensions, 'rectangle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = RectangleDimensions.Request()

    def send_request(self, length, breadth):
        self.req.length = length
        self.req.breadth = breadth
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    client = RectangleClient()
    length = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    breadth = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    client.send_request(length, breadth)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            else:
                client.get_logger().info(f'Area received: {response.area}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
