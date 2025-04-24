# GPIO Manager Node

This ROS2 node provides an interface to control GPIO pins on a Raspberry Pi 5. It supports reading and writing to specific GPIO pins through ROS2 services.

## Important Note for Raspberry Pi 5 Users

The Raspberry Pi 5 uses a new RP1 chip for GPIO control, which is not yet fully supported by the pigpio library. This node currently uses pigpio, which may not work correctly on the Pi 5. We recommend one of the following alternatives:

1. Use the newer libgpiod library (recommended for Pi 5):
```bash
sudo apt-get update
sudo apt-get install libgpiod-dev libgpiod-doc
```

2. Use the standard Linux GPIO interface through sysfs (legacy method):
```bash
sudo apt-get update
sudo apt-get install gpiod
```

We are working on updating this node to use libgpiod for better Pi 5 compatibility. Please check back for updates.

## Supported GPIO Pins

The node supports the following GPIO pins:
- 13
- 16
- 18
- 19
- 20
- 21
- 22
- 23
- 24

## Usage

### Launching the Node

```bash
ros2 launch dexi_cpp gpio_manager.launch.py
```

### Setting GPIO State

To set a GPIO pin high or low, use the `/set_gpio` service:

```bash
# Set pin 13 high
ros2 service call /set_gpio gpio_manager/srv/SetGpio "{pin: 13, value: true}"

# Set pin 13 low
ros2 service call /set_gpio gpio_manager/srv/SetGpio "{pin: 13, value: false}"
```

### Reading GPIO State

To read the current state of a GPIO pin, use the `/read_gpio` service:

```bash
# Read state of pin 13
ros2 service call /read_gpio gpio_manager/srv/ReadGpio "{pin: 13}"
```

The service will return:
- `value`: true if the pin is high, false if low
- `success`: true if the operation was successful
- `message`: A status message describing the result

### Controlling Servos

To control a servo motor, use the `/servo_control` service:

```bash
# Move servo on GPIO 13 to 90 degrees (center position)
ros2 service call /servo_control dexi_interfaces/srv/ServoControl "{pin: 13, angle: 90.0}"

# Move servo on GPIO 13 to 0 degrees (full left)
ros2 service call /servo_control dexi_interfaces/srv/ServoControl "{pin: 13, angle: 0.0}"

# Move servo on GPIO 13 to 180 degrees (full right)
ros2 service call /servo_control dexi_interfaces/srv/ServoControl "{pin: 13, angle: 180.0}"
```

The service accepts the following parameters:
- `pin`: The GPIO pin number the servo is connected to
- `angle`: The desired angle (0-180 degrees)
- `min_pw`: (Optional) Minimum pulse width in microseconds (default: 500)
- `max_pw`: (Optional) Maximum pulse width in microseconds (default: 2500)

The service will return:
- `success`: true if the operation was successful
- `message`: A status message describing the result

## Installation and Setup

### Installing Required Libraries

For Raspberry Pi 4 and earlier:
```bash
sudo apt-get update
sudo apt-get install pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

For Raspberry Pi 5 (using libgpiod):
```bash
sudo apt-get update
sudo apt-get install libgpiod-dev libgpiod-doc
```

### Troubleshooting

If you encounter permission issues, you may need to add your user to the `gpio` group:
```bash
sudo usermod -a -G gpio $USER
```

After adding to the group, you'll need to log out and log back in for the changes to take effect.

### Notes

- For Raspberry Pi 5 users: This node is currently being updated to use libgpiod for better compatibility
- All pins are initialized as outputs by default
- The node will validate pin numbers and return an error if an unsupported pin is requested 