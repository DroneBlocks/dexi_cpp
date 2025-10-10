# DEXI C++ ROS2 Nodes

A collection of C++ ROS2 nodes for the DEXI drone project, providing GPIO control, servo management, and I2C device interfaces.

## Nodes Overview

- **Servo Controller**: Controls servos via PCA9685 PWM controller
- **TCA9555 Controller**: Manages TCA9555 16-bit I2C I/O expander for GPIO control
- **RGB Status LED Controller**: Controls RGB status indicators

## Prerequisites

### System Dependencies

```bash
# I2C support (required for TCA9555 and PCA9685)
sudo apt-get update
sudo apt-get install i2c-tools
sudo usermod -a -G i2c $USER
```

### I2C Setup

1. Enable I2C in raspi-config:
```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
```

2. Create udev rules for I2C access:
```bash
sudo bash -c 'echo "SUBSYSTEM==\"i2c-dev\", GROUP=\"i2c\", MODE=\"0660\"" > /etc/udev/rules.d/90-i2c.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. Verify I2C devices:
```bash
i2cdetect -y 1
```

## Building

```bash
# Build the package
colcon build --packages-select dexi_cpp --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Servo Controller

Controls servos via PCA9685 PWM controller with robust I2C communication and comprehensive error handling.

**Hardware Setup:**
- Connect PCA9685 to I2C (SDA: GPIO 2, SCL: GPIO 3)
- Connect servos to PWM channels 0-15 (all 16 channels supported)
- Power servos from 5V supply
- PCA9685 uses I2C address 0x40 by default

**Features:**
- Supports all 16 PCA9685 channels (0-15)
- Custom pulse width control per servo
- Input validation and error handling
- 50Hz PWM frequency optimized for servos

```bash
ros2 launch dexi_cpp servo_controller.launch.py
```

**Services:**
- `/dexi/servo_control` (dexi_interfaces/srv/ServoControl) - Control servo position

**Service Interface:**
```
# Request fields:
int32 pin      # Servo channel (0-15)
int32 angle    # Target angle in degrees (0-180)
int32 min_pw   # Optional: minimum pulse width in microseconds (default: 500)
int32 max_pw   # Optional: maximum pulse width in microseconds (default: 2500)
---
# Response fields:
bool success   # Operation success status
string message # Status or error message
```

**Usage Examples:**
```bash
# Move servo on channel 0 to 90 degrees (using defaults: 500-2500μs pulse width)
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: 0, angle: 90}"

# Move servo on channel 1 to 0 degrees
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: 1, angle: 0}"

# Move servo on channel 2 to 180 degrees
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: 2, angle: 180}"

# Use custom pulse width range for specific servo (e.g., 600-2400μs)
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: 0, angle: 90, min_pw: 600, max_pw: 2400}"

# Control servo on channel 5 (demonstrates 16-channel support)
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: 5, angle: 45}"
```

**Testing Multiple Servos:**
```bash
# Sequential servo control - sweep through channels 0-4
for i in {0..4}; do
  ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: $i, angle: 90}"
  sleep 0.5
done

# Center all servos
for i in {0..4}; do
  ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl "{pin: $i, angle: 90}"
done
```

### TCA9555 Controller

Manages TCA9555 16-bit I2C I/O expander for GPIO control with configurable pin modes.

**Hardware Setup:**
- Connect TCA9555 to I2C (SDA: GPIO 2, SCL: GPIO 3)
- Set address pins A0, A1, A2 to GND for address 0x20
- Power from 3.3V
- **Note**: Supports pins 0-4 and 14-15

```bash
ros2 launch dexi_cpp tca9555_controller.launch.py
```

**Configuration:**
The launch file configures pins 0-2 and 14-15 as outputs and pins 3-4 as inputs by default.

**Services:**
- `/dexi/gpio_writer_service/write_gpio` (dexi_interfaces/srv/GPIOSend) - Set output pin state

**Topics:**
- `/tca9555_controller/gpio_<pin>_state` (std_msgs/Bool) - Input pin states

**Usage:**
```bash
# Set output pin 0 high
ros2 service call /dexi/gpio_writer_service/write_gpio dexi_interfaces/srv/GPIOSend "{pin: 0, state: true}"

# Set output pin 0 low
ros2 service call /dexi/gpio_writer_service/write_gpio dexi_interfaces/srv/GPIOSend "{pin: 0, state: false}"

# Monitor input pin 3
ros2 topic echo /tca9555_controller/gpio_3_state
```

### RGB Status LED Controller

Controls RGB status indicators.

```bash
ros2 launch dexi_cpp rgb_status_led_controller.launch.py
```

## Configuration

### TCA9555 Pin Configuration

Modify pin modes in `launch/tca9555_controller.launch.py`:

```python
'pin_0_mode': True,   # Pin 0: output
'pin_1_mode': True,   # Pin 1: output
'pin_2_mode': True,   # Pin 2: output
'pin_3_mode': False,  # Pin 3: input
'pin_4_mode': False,  # Pin 4: input
'pin_14_mode': True,  # Pin 14: output
'pin_15_mode': True   # Pin 15: output
```

### Parameter Overrides

Override parameters at launch:

```bash
# Change I2C device and polling rate
ros2 launch dexi_cpp tca9555_controller.launch.py i2c_device:=/dev/i2c-0 input_polling_rate:=5.0

# Change servo parameters
ros2 launch dexi_cpp servo_controller.launch.py i2c_address:=64
```

## Troubleshooting

### Permission Issues
```bash
# Add user to i2c group (required for I2C devices)
sudo usermod -a -G i2c $USER

# Log out and back in for group changes to take effect
```

### I2C Issues
```bash
# Check I2C devices
i2cdetect -y 1

# Check I2C permissions
ls -la /dev/i2c-*

# Test I2C communication
i2cget -y 1 0x20 0x00  # Read TCA9555 input port 0
```

### Build Issues
```bash
# Clean and rebuild
rm -rf build/dexi_cpp install/dexi_cpp log/latest_build/dexi_cpp
colcon build --packages-select dexi_cpp --symlink-install
source install/setup.bash
```

## Examples

### Pin Control Example

A Python example demonstrating how to read TCA9555 input pins and control output pins:

```bash
# Make executable
chmod +x examples/example_pin_control.py

# Run the example (requires TCA9555 controller to be running)
python3 examples/example_pin_control.py
```

This example subscribes to pin 4 input and mirrors its state to pin 0 output.

### Toggle Pin Example

A simple example that toggles TCA9555 pin 14 on and off at 1-second intervals:

```bash
# Make executable
chmod +x examples/example_toggle_pin.py

# Run the example (requires TCA9555 controller to be running)
python3 examples/example_toggle_pin.py
```

## Dependencies

- **ROS2 Humble** or later
- **rclcpp** - ROS2 C++ client library
- **std_msgs** - Standard ROS2 message types
- **dexi_interfaces** - Custom service and message definitions

## License

MIT License - see LICENSE file for details.