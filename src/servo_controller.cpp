#include "dexi_cpp/servo_controller.hpp"
#include <functional>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

namespace dexi_cpp {

ServoController::ServoController() : Node("servo_controller")
{
    RCLCPP_INFO(get_logger(), "Initializing DEXI Servo Controller with PCA9685...");

    // Initialize PCA9685 device
    if (!initializePca9685()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PCA9685 device");
        return;
    }

    // Create service for servo control using DEXI's existing interface
    servo_service_ = create_service<dexi_interfaces::srv::ServoControl>(
        "/dexi/servo_control",
        std::bind(&ServoController::servoControlCallback, this, _1, _2)
    );

    RCLCPP_INFO(get_logger(), "DEXI Servo Controller initialized successfully");
    RCLCPP_INFO(get_logger(), "Service available at: /dexi/servo_control");
}

ServoController::~ServoController()
{
    RCLCPP_INFO(get_logger(), "Shutting down DEXI Servo Controller");
    // PCA9685 destructor will handle cleanup
}

bool ServoController::initializePca9685()
{
    try {
        // Create PCA9685 device instance (using defaults: /dev/i2c-1, address 0x40, gen call 0x00)
        pca9685_dev_ = std::make_unique<Pca9685>();

        // Set servo frequency (50Hz standard)
        auto ret = pca9685_dev_->Pca9685_SetPrescale(DEFAULT_SERVO_FREQ, PCA9685_OSC_HZ);
        if (ret != Pca9685Hal::PCA9685_OK) {
            RCLCPP_ERROR(get_logger(), "Failed to set PCA9685 prescale");
            return false;
        }

        // Set to normal power mode
        ret = pca9685_dev_->Pca9685_PwrMode(Pca9685::PCA9685_MODE_NORMAL);
        if (ret != Pca9685Hal::PCA9685_OK) {
            RCLCPP_ERROR(get_logger(), "Failed to set PCA9685 power mode");
            return false;
        }

        RCLCPP_INFO(get_logger(), "PCA9685 initialized successfully");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception during PCA9685 initialization: %s", e.what());
        return false;
    }
}

void ServoController::servoControlCallback(
    const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
    std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response)
{
    try {
        // Validate input parameters
        if (!isValidChannel(request->pin)) {
            response->success = false;
            response->message = "Invalid channel number. Must be between 0 and " + std::to_string(MAX_CHANNEL_NUM);
            RCLCPP_WARN(get_logger(), "Invalid channel: %d", request->pin);
            return;
        }

        if (!isValidAngle(request->angle)) {
            response->success = false;
            response->message = "Invalid angle. Must be between 0 and " + std::to_string(static_cast<int>(MAX_SERVO_ANGLE)) + " degrees";
            RCLCPP_WARN(get_logger(), "Invalid angle: %d", request->angle);
            return;
        }

        // Use provided pulse width values or defaults
        double min_pw_us = (request->min_pw > 0) ? request->min_pw : DEFAULT_MIN_PW_US;
        double max_pw_us = (request->max_pw > 0) ? request->max_pw : DEFAULT_MAX_PW_US;

        // Clamp angle to valid range
        double angle = std::clamp(static_cast<double>(request->angle), 0.0, MAX_SERVO_ANGLE);

        // Calculate duty cycle for the servo
        double duty_cycle = calculateDutyCycle(angle, min_pw_us, max_pw_us);

        // Set PWM output on the specified channel
        auto ret = pca9685_dev_->Pca9685_LedPwmSet(
            static_cast<uint8_t>(request->pin),
            duty_cycle
        );

        if (ret != Pca9685Hal::PCA9685_OK) {
            response->success = false;
            response->message = "Failed to set PWM output on PCA9685";
            RCLCPP_ERROR(get_logger(), "PCA9685 PWM set failed for channel %d", request->pin);
            return;
        }

        // Success!
        response->success = true;
        response->message = "Servo position updated successfully";

        RCLCPP_INFO(get_logger(),
            "Updated servo channel %d to %.1f degrees (PWM: %.1f%%, Min: %.0fμs, Max: %.0fμs)",
            request->pin, angle, duty_cycle, min_pw_us, max_pw_us);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in servo control callback: %s", e.what());
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}

double ServoController::calculateDutyCycle(double angle, double min_pw_us, double max_pw_us) const
{
    // Convert angle (0-180°) to pulse width in microseconds
    double pulse_width_us = min_pw_us + (max_pw_us - min_pw_us) * (angle / MAX_SERVO_ANGLE);

    // Convert pulse width to duty cycle percentage
    // For 50Hz: period = 20ms = 20000μs
    double period_us = 1000000.0 / DEFAULT_SERVO_FREQ;  // Period in microseconds
    double duty_cycle = (pulse_width_us / period_us) * 100.0;  // Convert to percentage

    return duty_cycle;
}

bool ServoController::isValidChannel(int32_t pin) const
{
    return (pin >= 0 && pin <= MAX_CHANNEL_NUM);
}

bool ServoController::isValidAngle(int32_t angle) const
{
    return (angle >= 0 && angle <= static_cast<int32_t>(MAX_SERVO_ANGLE));
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