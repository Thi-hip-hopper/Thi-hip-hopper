import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MultiValuePublisher(Node):
    def __init__(self):
        super().__init__('multi_value_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.val1 = 0
        self.val2 = 100
        self.val3 = 1000

    def timer_callback(self):
        msg = String()
        # Create a message with three incrementing values
        msg.data = f'{self.val1},{self.val2},{self.val3}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment each value
        self.val1 += 1
        self.val2 += 2
        self.val3 += 3

def main(args=None):
    rclpy.init(args=args)
    node = MultiValuePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
