#include "dexi_cpp/servo_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>
#include <functional>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

// Servo parameters
static constexpr int PWM_FREQUENCY = 50;  // 50Hz standard servo frequency
static constexpr int MIN_PULSE_WIDTH = 500;  // 0.5ms (0 degrees)
static constexpr int MAX_PULSE_WIDTH = 2500; // 2.5ms (180 degrees)
static constexpr int MID_PULSE_WIDTH = 1500; // 1.5ms (90 degrees)
static constexpr int SERVO_PIN = 13; // Assuming SERVO_PIN is defined in the header file

namespace dexi_cpp
{

ServoController::ServoController()
: Node("servo_controller")
{
    // Declare and get parameters
    this->declare_parameter("servo_pins", std::vector<int64_t>{13});  // Default to pin 13
    auto servo_pins_param = this->get_parameter("servo_pins").as_integer_array();
    
    // Convert to vector of int
    for (const auto& pin : servo_pins_param) {
        servo_pins_.push_back(static_cast<int>(pin));
    }

    // Open GPIO chip
    gpio_handle_ = lgGpiochipOpen(0);
    if (gpio_handle_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open GPIO chip");
        return;
    }
    RCLCPP_INFO(get_logger(), "Successfully opened GPIO chip");

    // Initialize servo pins - continue even if some pins fail
    initializeServoPins();

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
    if (gpio_handle_ >= 0) {
        lgGpiochipClose(gpio_handle_);
    }
}

bool ServoController::initializeServoPins()
{
    bool any_success = false;
    for (int pin : servo_pins_) {
        // Claim GPIO for output
        if (lgGpioClaimOutput(gpio_handle_, 0, pin, 0) != LG_OKAY) {
            RCLCPP_ERROR(get_logger(), "Failed to claim GPIO %d for output", pin);
            continue;
        }

        // Start servo at center position
        if (lgTxServo(gpio_handle_, pin, MID_PULSE_WIDTH, PWM_FREQUENCY, 0, 0) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize servo on GPIO %d", pin);
            // Release the pin if servo initialization fails
            lgGpioFree(gpio_handle_, pin);
            continue;
        }

        RCLCPP_INFO(get_logger(), "Successfully initialized servo on GPIO %d", pin);
        any_success = true;
    }

    if (!any_success) {
        RCLCPP_ERROR(get_logger(), "No servo pins were successfully initialized");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Successfully initialized %zu servo pins", servo_pins_.size());
    return true;
}

void ServoController::servoControlCallback(
    const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
    std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response)
{
    try {
        // Check if requested pin is in our list of servo pins
        auto it = std::find(servo_pins_.begin(), servo_pins_.end(), request->pin);
        if (it == servo_pins_.end()) {
            response->success = false;
            response->message = "Pin " + std::to_string(request->pin) + " is not configured for servo control";
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
        if (lgTxServo(gpio_handle_, request->pin, pulse_width, PWM_FREQUENCY, 0, 0) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to update servo position on GPIO %d", request->pin);
            response->success = false;
            response->message = "Failed to update servo position";
            return;
        }

        response->success = true;
        response->message = "Servo position updated successfully";
        RCLCPP_INFO(get_logger(), "Updated servo on GPIO %d to angle %f (pulse width: %d)", 
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