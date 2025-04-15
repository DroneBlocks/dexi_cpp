from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_cpp',
            executable='gpio_manager',
            name='gpio_manager',
            output='screen',
            parameters=[],
            remappings=[],
        )
    ]) 