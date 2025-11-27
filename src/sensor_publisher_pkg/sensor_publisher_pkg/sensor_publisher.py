import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        # Publishes a Float32 message on the /sensor_value topic
        self.publisher_ = self.create_publisher(Float32, 'sensor_value', 10)
        # The timer is triggered every 0.1 seconds (10 Hz)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Sensor Publisher Node has been started.')

    def timer_callback(self):
       # Generate random synthetic sensor data (between 0 and 20)
        sensor_data = random.uniform(0.0, 20.0) 
        msg = Float32()
        msg.data = sensor_data
        
        # Publish data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing sensor value: "%f"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
