import rclpy
from rclpy.node import Node
from my_interfaces.srv import RectangleDimensions

class RectangleServer(Node):
    def __init__(self):
        super().__init__('rectangle_server')
        self.srv = self.create_service(RectangleDimensions, 'rectangle', self.calculate_area)

    def calculate_area(self, request, response):
        length = request.length
        breadth = request.breadth
        area = length * breadth
        response.area = area
        self.get_logger().info(f'length recieved is: {length}, breadth: {breadth}, area calculated is : {area}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RectangleServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
