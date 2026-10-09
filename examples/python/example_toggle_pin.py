#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import SetGpio
import time


class TogglePinNode(Node):
    """
    Simple example that toggles TCA9555 pin 14 on and off at 1-second intervals.
    """
    
    def __init__(self):
        super().__init__('toggle_pin_node')
        
        # Create service client for controlling TCA9555 output
        self.tca9555_client = self.create_client(
            SetGpio, 
            '/tca9555_controller/set_tca9555_pin'
        )
        
        # Wait for service to be available
        while not self.tca9555_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TCA9555 service...')
        
        # Initialize pin state
        self.pin_state = False
        self.pin_number = 14  # Toggle pin 14
        
        # Create timer for toggling (1 second interval)
        self.timer = self.create_timer(1.0, self.toggle_callback)
        
        self.get_logger().info(f'Toggle pin node started - toggling pin {self.pin_number} every 1 second')
    
    def toggle_callback(self):
        """
        Timer callback that toggles the pin state.
        """
        # Toggle the state
        self.pin_state = not self.pin_state
        
        # Create service request
        request = SetGpio.Request()
        request.pin = self.pin_number
        request.value = self.pin_state
        
        # Send request asynchronously
        future = self.tca9555_client.call_async(request)
        future.add_done_callback(self.service_callback)
        
        # Log the action
        state_str = "HIGH" if self.pin_state else "LOW"
        self.get_logger().info(f'Setting pin {self.pin_number} to {state_str}')
    
    def service_callback(self, future):
        """
        Callback for service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug('Successfully set pin')
            else:
                self.get_logger().error(f'Failed to set pin: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TogglePinNode()
    
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