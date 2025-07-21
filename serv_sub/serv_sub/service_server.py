import rclpy
from rclpy.node import Node
from serv_sub_interfaces.srv import ReqRequest  
class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')


        self.srv = self.create_service(ReqRequest, 'subscriber', self.handle_request)
        self.get_logger().info('Service "subscriber" is ready.')

    def handle_request(self, request, response):
        self.get_logger().info(f'Received service request with input: {request.input}')

        response.output = f'Acknowledged: {request.input}'

        self.get_logger().info(f'Sending response: {response.output}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = ServiceServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
