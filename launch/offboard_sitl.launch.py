import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Micro XRCE-DDS Agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
            output='screen'
        ),

        # Start px4_offboard_manager node
        Node(
            package='dexi_cpp',
            executable='px4_offboard_manager',
            name='offboard_manager',
            namespace='dexi',
            output='screen'
        )
    ]) 