from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_cpp',
            executable='tca9555_controller',
            name='tca9555_controller',
            output='screen',
            parameters=[{
                'i2c_device': '/dev/i2c-1',
                'i2c_address': 32,  # 0x20 in decimal
                'available_pins': [0, 1, 2, 3, 4],
                'input_polling_rate': 1.0,  # Hz
                'pin_modes': {
                    '0': True,   # Pin 0: output (True = output, False = input)
                    '1': True,   # Pin 1: output
                    '2': True,   # Pin 2: output
                    '3': False,  # Pin 3: input
                    '4': False   # Pin 4: input
                }
            }]
        )
    ]) 