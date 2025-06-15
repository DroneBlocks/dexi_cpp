from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_cpp',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            emulate_tty=True
        )
    ]) 