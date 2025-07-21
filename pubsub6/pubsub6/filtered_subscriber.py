import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self, my_id):
        super().__init__(f'subscriber_{my_id}')
        self.subscription = self.create_subscription(String,'/simon',self.listener_callback,10)
        self.publisher = self.create_publisher(String, '/execute_command', 10)


    def listener_callback(self, msg):
        self.get_logger().info(f'Received command on /simon: simon says "{msg.data}"')
        self.publisher.publish(msg)
        self.get_logger().info(f'Forwarded command on /execute_command')

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
