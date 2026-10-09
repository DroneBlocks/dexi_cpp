#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from dexi_interfaces.srv import SetGpio


class PinControlNode(Node):
    """
    Example node that reads TCA9555 pin 4 input and controls pin 0 output.
    When pin 4 is HIGH, pin 0 is set HIGH. When pin 4 is LOW, pin 0 is set LOW.
    """
    
    def __init__(self):
        super().__init__('pin_control_node')
        
        # Create service client for controlling TCA9555 output
        self.tca9555_client = self.create_client(
            SetGpio, 
            '/tca9555_controller/set_tca9555_pin'
        )
        
        # Wait for service to be available
        while not self.tca9555_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TCA9555 service...')
        
        # Create subscription to pin 4 state
        self.pin4_subscription = self.create_subscription(
            Bool,
            '/tca9555_controller/gpio_4_state',
            self.pin4_callback,
            10
        )
        
        self.get_logger().info('Pin control node started')
        self.get_logger().info('Subscribing to pin 4 input, controlling pin 0 output')
    
    def pin4_callback(self, msg):
        """
        Callback function called when pin 4 state changes.
        Controls pin 0 output based on pin 4 input state.
        """
        pin4_state = msg.data
        pin0_state = pin4_state  # Mirror the state
        
        # Create service request
        request = SetGpio.Request()
        request.pin = 0  # Control pin 0
        request.value = pin0_state
        
        # Send request asynchronously
        future = self.tca9555_client.call_async(request)
        future.add_done_callback(self.service_callback)
        
        # Log the action
        state_str = "HIGH" if pin4_state else "LOW"
        self.get_logger().info(f'Pin 4 is {state_str} -> Setting pin 0 to {state_str}')
    
    def service_callback(self, future):
        """
        Callback for service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug('Successfully set pin 0')
            else:
                self.get_logger().error(f'Failed to set pin 0: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = PinControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors


if __name__ == '__main__':
    main() 