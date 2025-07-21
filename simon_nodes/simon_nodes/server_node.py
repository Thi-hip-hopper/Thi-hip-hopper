import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.subscriber = self.create_subscription(String,'/execute_command',self.execute_command_callback,10)

    def execute_command_callback(self, msg):
        full_command = msg.data.strip()
        self.get_logger().info(f"Received full command sequence: '{full_command}'")

        
        commands = full_command.lower().split()

        for action in commands:
            if action == "forward":
                self.get_logger().info("Moving forward")
            elif action == "backward":
                self.get_logger().info("Moving backward")
            elif action == "right":
                self.get_logger().info("Turning right")
            elif action == "left":
                self.get_logger().info("Turning left")
            else:
                self.get_logger().warn(f"Unknown action: '{action}'")
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
