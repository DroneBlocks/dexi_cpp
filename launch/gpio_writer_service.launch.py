from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_cpp',
            executable='gpio_writer_service',
            name='gpio_writer_service',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'gpio_pins': [23, 24]  # Configure GPIO output pins
            }]
        )
    ]) 