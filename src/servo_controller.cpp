#include "dexi_cpp/servo_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

// Servo parameters
static constexpr int PWM_FREQUENCY = 50;  // 50Hz standard servo frequency
static constexpr int MIN_PULSE_WIDTH = 500;  // 0.5ms (0 degrees)
static constexpr int MAX_PULSE_WIDTH = 2500; // 2.5ms (180 degrees)
static constexpr int MID_PULSE_WIDTH = 1500; // 1.5ms (90 degrees)

namespace dexi_cpp
{

ServoController::ServoController()
: Node("servo_controller")
{
    // Open I2C bus
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open I2C bus");
        return;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd_, I2C_SLAVE, PCA9685_ADDR) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to set I2C slave address");
        close(i2c_fd_);
        return;
    }

    // Initialize PCA9685
    if (!initializePCA9685()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PCA9685");
        close(i2c_fd_);
        return;
    }

    // Create service for servo control
    servo_service_ = create_service<dexi_interfaces::srv::ServoControl>(
        "servo_control",
        std::bind(&ServoController::servoControlCallback, this, _1, _2)
    );
    RCLCPP_INFO(get_logger(), "Created service for servo control");
}

ServoController::~ServoController()
{
    RCLCPP_INFO(get_logger(), "Shutting down servo controller");
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

bool ServoController::initializePCA9685()
{
    try {
        // Wake up PCA9685 (clear sleep)
        writeByte(MODE1, 0x00);

        // Set PWM frequency to 50Hz for servos
        writeByte(MODE1, 0x10); // sleep
        writeByte(PRESCALE, 121); // 50Hz
        writeByte(MODE1, 0x00); // wake
        usleep(5000);
        writeByte(MODE1, 0xA1); // auto-increment on

        // Initialize all servos to center position
        for (int i = 0; i < 4; i++) {
            setPWM(i, 0, MID_PULSE_WIDTH);
        }

        RCLCPP_INFO(get_logger(), "Successfully initialized PCA9685");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PCA9685: %s", e.what());
        return false;
    }
}

void ServoController::writeByte(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        throw std::runtime_error("Failed to write to I2C device");
    }
}

void ServoController::setPWM(int channel, int on, int off)
{
    writeByte(LED0_ON_L + 4 * channel, on & 0xFF);
    writeByte(LED0_ON_L + 4 * channel + 1, on >> 8);
    writeByte(LED0_ON_L + 4 * channel + 2, off & 0xFF);
    writeByte(LED0_ON_L + 4 * channel + 3, off >> 8);
}

void ServoController::servoControlCallback(
    const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
    std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response)
{
    try {
        // Check if requested servo index is valid (0-3)
        if (request->pin < 0 || request->pin > 3) {
            response->success = false;
            response->message = "Invalid servo index. Must be between 0 and 3";
            return;
        }

        // Calculate pulse width based on angle
        int min_pw = request->min_pw > 0 ? request->min_pw : MIN_PULSE_WIDTH;
        int max_pw = request->max_pw > 0 ? request->max_pw : MAX_PULSE_WIDTH;
        
        // Clamp angle to valid range
        double angle = std::max(0.0, std::min(180.0, static_cast<double>(request->angle)));
        
        // Calculate pulse width
        int pulse_width = static_cast<int>(
            min_pw + (max_pw - min_pw) * (angle / 180.0)
        );

        // Update servo position
        setPWM(request->pin, 0, pulse_width);

        response->success = true;
        response->message = "Servo position updated successfully";
        RCLCPP_INFO(get_logger(), "Updated servo %d to angle %f (pulse width: %d)", 
            request->pin, angle, pulse_width);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in servo control callback: %s", e.what());
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}

} // namespace dexi_cpp

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_cpp::ServoController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 