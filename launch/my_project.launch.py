from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. sensor_publisher node
	# Starts the sensor_publisher executable from the publisher package 				(sensor_publisher_pkg)
        Node(
            package='sensor_publisher_pkg',
            executable='sensor_publisher',
            name='sensor_publisher_node',
            output='screen'
        ),
        
        # 2. data_processor node
	# Starts the data_processor executable from the processor package (data_processor_pkg)
        Node(
            package='data_processor_pkg',
            executable='data_processor',
            name='data_processor_node',
            output='screen'
        ),
        
        # 3. command_server node
	# Starts the command_server executable from the service server package (command_server_pkg)
        Node(
            package='command_server_pkg',
            executable='command_server',
            name='command_server_node',
            output='screen'
        ),
    ])
