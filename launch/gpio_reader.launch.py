from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_cpp',
            executable='gpio_reader',
            name='gpio_reader',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'gpio_pins': [20, 21, 22]  # Configure GPIO input pins
            }]
        )
    ]) 