#!/usr/bin/env python3
"""Send raw PWM pulse widths (microseconds) to the servo_controller node.

The /dexi/servo_control service is angle-based, but by setting
min_pw == max_pw == desired_pw the angle term collapses and the node
writes exactly that pulse width to the PCA9685 channel.
"""

import sys
import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import ServoControl


class ServoPwmClient(Node):
    def __init__(self):
        super().__init__('servo_pwm_client')
        self.cli = self.create_client(ServoControl, '/dexi/servo_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dexi/servo_control...')

    def set_pwm(self, channel: int, pulse_us: int) -> bool:
        req = ServoControl.Request()
        req.pin = channel
        req.angle = 0                # collapses out when min_pw == max_pw
        req.min_pw = pulse_us
        req.max_pw = pulse_us
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        self.get_logger().info(
            f'ch={channel} pw={pulse_us}us -> success={resp.success} '
            f'msg="{resp.message}"'
        )
        return resp.success


def main():
    # Usage: example_servo_pwm.py <channel> <pulse_us>
    # Defaults: channel 0, 1500us (servo center)
    channel = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    pulse_us = int(sys.argv[2]) if len(sys.argv) > 2 else 1500

    rclpy.init()
    node = ServoPwmClient()
    try:
        node.set_pwm(channel, pulse_us)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
