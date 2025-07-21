import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        self.subscriber = self.create_subscription(String,'/simon',self.listener_callback,10)
        self.publisher = self.create_publisher(String, '/execute_command', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command on /simon: simon says "{msg.data}"')
        self.publisher.publish(msg)
        self.get_logger().info(f'Forwarded command on /execute_command')

def main(args=None):
    rclpy.init(args=args)
    relay_node = RelayNode()
    rclpy.spin(relay_node)
    relay_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
