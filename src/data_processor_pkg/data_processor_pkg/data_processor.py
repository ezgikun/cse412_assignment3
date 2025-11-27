import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        # Publishes Float32 message on topic /processed_value [cite: 33]
        self.publisher_ = self.create_publisher(Float32, 'processed_value', 10)
        
        # Subscribes to topic /sensor_value [cite: 31]
        self.subscription = self.create_subscription(
            Float32,
            'sensor_value',
            self.listener_callback,
            10)
        self.get_logger().info('Data Processor Node has been started.')

    def listener_callback(self, msg):
        # 1. Processing the incoming data (Ex: Multiplying the data by 2) [cite: 35]
        processed_data = msg.data * 2.0
        
       # 2. Publishing processed data [cite: 32]
        processed_msg = Float32()
        processed_msg.data = processed_data
        self.publisher_.publish(processed_msg)
        
        # Logging
        self.get_logger().info('Processed: Raw: %f -> Result: %f' % (msg.data, processed_data))

def main(args=None):
    rclpy.init(args=args)
    data_processor = DataProcessor()
    rclpy.spin(data_processor)
    data_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
