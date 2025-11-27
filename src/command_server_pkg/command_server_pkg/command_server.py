import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ComputeCommand 

class CommandServer(Node):
    def __init__(self):
        super().__init__('command_server')
        self.srv = self.create_service(ComputeCommand, 'compute_command', self.compute_callback)
        self.get_logger().info('Command Server Node has been started. Service ready at /compute_command')

    def compute_callback(self, request, response):
        
        input_value = request.input
        
       
        if input_value > 10.0:
            response.output = "HIGH"
        else: # input <= 10
            response.output = "LOW"
            
        self.get_logger().info(f'Incoming Request: Input={input_value}. Responding with: {response.output}')
        
        
        return response

def main(args=None):
    rclpy.init(args=args)
    command_server = CommandServer()
    rclpy.spin(command_server)
    command_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

