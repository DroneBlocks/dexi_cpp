#!/usr/bin/env python3
"""Send angle commands (0-180 degrees) to the servo_controller node.

The /dexi/servo_control service maps angle -> pulse width using its
default 500-2500 us range. Pass min_pw / max_pw on the command line if
your servo needs a different range (e.g. 1000-2000 us for many ESCs).
"""

import argparse
import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import ServoControl


class ServoAngleClient(Node):
    def __init__(self):
        super().__init__('servo_angle_client')
        self.cli = self.create_client(ServoControl, '/dexi/servo_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dexi/servo_control...')

    def set_angle(self, channel: int, angle: int,
                  min_pw: int = 0, max_pw: int = 0) -> bool:
        req = ServoControl.Request()
        req.pin = channel
        req.angle = angle
        req.min_pw = min_pw   # 0 -> node uses default 500 us
        req.max_pw = max_pw   # 0 -> node uses default 2500 us
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        self.get_logger().info(
            f'ch={channel} angle={angle} -> success={resp.success} '
            f'msg="{resp.message}"'
        )
        return resp.success


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('channel', type=int, nargs='?', default=0,
                        help='PCA9685 channel (0-15, default 0)')
    parser.add_argument('angle', type=int, nargs='?', default=90,
                        help='Servo angle in degrees (0-180, default 90)')
    parser.add_argument('--min-pw', type=int, default=0,
                        help='Optional minimum pulse width in us')
    parser.add_argument('--max-pw', type=int, default=0,
                        help='Optional maximum pulse width in us')
    args = parser.parse_args()

    rclpy.init()
    node = ServoAngleClient()
    try:
        node.set_angle(args.channel, args.angle, args.min_pw, args.max_pw)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
