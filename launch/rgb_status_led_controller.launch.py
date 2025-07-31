from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_cpp',
            executable='rgb_status_led_controller',
            name='rgb_status_led_controller',
            output='screen',
            emulate_tty=True
        )
    ]) 