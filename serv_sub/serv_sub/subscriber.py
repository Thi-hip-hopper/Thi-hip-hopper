import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serv_sub_interfaces.srv import ReqRequest  
class Subscriber(Node):
    def __init__(self, my_id='1'):  
        super().__init__(f'subscriber_{my_id}')
        
        
        self.subscription = self.create_subscription(String,'/simon',self.listener_callback,10)

        
        self.publisher = self.create_publisher(String,'/execute_command',10)

        
        self.cli = self.create_client(ReqRequest, 'subscriber')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        
        self.req = ReqRequest.Request()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command on /simon: simon says \"{msg.data}\"')

        self.publisher.publish(msg)
        self.get_logger().info('Forwarded command on /execute_command')

    
        self.req.input = msg.data
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

        

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.output}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
