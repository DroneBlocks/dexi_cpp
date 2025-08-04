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
                'available_pins': [0, 1, 2, 3, 4, 14, 15],
                'input_polling_rate': 1.0,  # Hz
                'pin_0_mode': True,   # Pin 0: output (True = output, False = input)
                'pin_1_mode': True,   # Pin 1: output
                'pin_2_mode': True,   # Pin 2: output
                'pin_3_mode': False,  # Pin 3: input
                'pin_4_mode': False,  # Pin 4: input
                'pin_14_mode': True,  # Pin 14: output
                'pin_15_mode': True   # Pin 15: output
            }]
        )
    ]) 