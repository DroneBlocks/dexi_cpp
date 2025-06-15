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

### Controlling Servos with PCA9685

The servo controller node uses a PCA9685 PWM controller chip to control up to 4 servos. The servos are connected to the PCA9685's PWM channels 0-3.

#### Hardware Setup

1. Connect the PCA9685 to your Raspberry Pi:
   - SDA -> GPIO 2 (Pin 3)
   - SCL -> GPIO 3 (Pin 5)
   - VCC -> 3.3V or 5V (depending on your servo requirements)
   - GND -> Ground
   - V+ -> 5V (for servo power)
   - Servos -> PWM channels 0-3

2. Enable I2C on your Raspberry Pi:
```bash
sudo raspi-config
# Navigate to Interface Options -> I2C -> Enable
```

3. Install I2C tools:
```bash
sudo apt-get update
sudo apt-get install i2c-tools
```

4. Grant I2C access to non-root users:
```bash
# Add your user to the i2c group
sudo usermod -a -G i2c $USER

# Create udev rules for I2C
sudo bash -c 'echo "SUBSYSTEM==\"i2c-dev\", GROUP=\"i2c\", MODE=\"0660\"" > /etc/udev/rules.d/90-i2c.rules'

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Note: You may need to log out and log back in for the group changes to take effect
```

5. Verify the PCA9685 is detected:
```bash
i2cdetect -y 1
# Should show device at address 0x40
```

#### Launching the Servo Controller

```bash
ros2 launch dexi_cpp servo_controller.launch.py
```

#### Controlling Servos

To control a servo motor, use the `/servo_control` service:

```bash
# Move servo 0 to 90 degrees (center position)
ros2 service call /servo_control dexi_interfaces/srv/ServoControl "{pin: 0, angle: 90.0}"

# Move servo 0 to 0 degrees (full left)
ros2 service call /servo_control dexi_interfaces/srv/ServoControl "{pin: 0, angle: 0.0}"

# Move servo 0 to 180 degrees (full right)
ros2 service call /servo_control dexi_interfaces/srv/ServoControl "{pin: 0, angle: 180.0}"
```

The service accepts the following parameters:
- `pin`: The servo index (0-3) corresponding to the PCA9685 PWM channel
- `angle`: The desired angle (0-180 degrees)
- `min_pw`: (Optional) Minimum pulse width (default: 150)
- `max_pw`: (Optional) Maximum pulse width (default: 600)

The service will return:
- `success`: true if the operation was successful
- `message`: A status message describing the result

#### Servo Specifications

- PWM Frequency: 50Hz
- Pulse Width Range: 150-600 (0.5ms - 2.5ms)
- Center Position: 375 (1.5ms)
- Supported Servos: Up to 4 servos (indices 0-3)

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

### Building and Running as Different Users

When switching between users (e.g., from a non-root user to root), you need to rebuild and reinstall the package to ensure the correct binaries are used. Here's the process:

1. Clean just the dexi_cpp build files:
```bash
# Remove only dexi_cpp build files
rm -rf build/dexi_cpp install/dexi_cpp log/latest_build/dexi_cpp
```

2. Rebuild just dexi_cpp:
```bash
# As the user who will run the node
colcon build --packages-select dexi_cpp --symlink-install
source install/setup.bash
```

3. Verify the installation:
```bash
# Check if the node is installed correctly
ros2 pkg list | grep dexi_cpp
ros2 pkg prefix dexi_cpp
```

4. Run the node:
```bash
ros2 launch dexi_cpp servo_controller.launch.py
```

Note: If you're still seeing old code being executed, try:
- Checking the installation path: `ros2 pkg prefix dexi_cpp`
- Verifying the binary location: `which servo_controller`
- Ensuring you've sourced the correct setup file: `source install/setup.bash`

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
- I2C access requires root privileges by default. Make sure to follow the I2C setup instructions to grant access to non-root users