#include "dexi_cpp/gpio_reader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <lgpio.h>

using std::placeholders::_1;

namespace dexi_cpp
{

GPIOReader::GPIOReader()
: Node("gpio_reader")
{
    // Declare and get parameters
    this->declare_parameter("gpio_pins", std::vector<int64_t>{20, 21, 22});  // Default pins
    auto gpio_pins_param = this->get_parameter("gpio_pins").as_integer_array();
    
    // Convert to vector of int
    for (const auto& pin : gpio_pins_param) {
        gpio_pins_.push_back(static_cast<int>(pin));
    }

    // Open GPIO chip
    gpio_handle_ = lgGpiochipOpen(0);
    if (gpio_handle_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open GPIO chip");
        return;
    }
    RCLCPP_INFO(get_logger(), "Successfully opened GPIO chip");

    // Initialize GPIO
    if (!initializeGpio()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize GPIO");
        return;
    }

    // Create publishers for each GPIO pin
    for (size_t i = 0; i < gpio_pins_.size(); ++i) {
        int pin = gpio_pins_[i];
        auto publisher = create_publisher<std_msgs::msg::Bool>(
            "~/gpio_" + std::to_string(pin),
            10
        );
        gpio_publishers_.push_back(publisher);
        RCLCPP_INFO(get_logger(), "Created publisher for GPIO %d", pin);
    }

    // Create timer for reading GPIO states
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GPIOReader::timerCallback, this)
    );
}

GPIOReader::~GPIOReader()
{
    RCLCPP_INFO(get_logger(), "Shutting down GPIO reader");
    cleanupGpio();
    if (gpio_handle_ >= 0) {
        lgGpiochipClose(gpio_handle_);
    }
}

bool GPIOReader::initializeGpio()
{
    try {
        // Initialize GPIO pins
        for (size_t i = 0; i < gpio_pins_.size(); ++i) {
            int pin = gpio_pins_[i];
            
            // Claim as input with pull-up enabled
            if (lgGpioClaimInput(gpio_handle_, LG_SET_PULL_UP, pin) != LG_OKAY) {
                RCLCPP_ERROR(get_logger(), "Failed to claim GPIO %d for input", pin);
                continue;
            }
            
            RCLCPP_INFO(get_logger(), "Successfully initialized GPIO %d", pin);
        }

        if (gpio_pins_.empty()) {
            RCLCPP_ERROR(get_logger(), "No GPIO pins were successfully initialized");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Successfully initialized %zu GPIO pins", gpio_pins_.size());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "GPIO initialization failed: %s", e.what());
        return false;
    }
}

void GPIOReader::cleanupGpio()
{
    RCLCPP_INFO(get_logger(), "Cleaning up GPIO pins");
    // Release all GPIO pins
    for (int pin : gpio_pins_) {
        lgGpioFree(gpio_handle_, pin);
    }
    gpio_pins_.clear();
    RCLCPP_INFO(get_logger(), "GPIO cleanup complete");
}

void GPIOReader::timerCallback()
{
    try {
        // Read and publish state for each GPIO pin
        for (size_t i = 0; i < gpio_pins_.size(); ++i) {
            int pin = gpio_pins_[i];
            auto msg = std::make_unique<std_msgs::msg::Bool>();
            
            // Read GPIO value
            int value = lgGpioRead(gpio_handle_, pin);
            if (value < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to read GPIO %d", pin);
                continue;
            }
            
            msg->data = (value != 0);
            gpio_publishers_[i]->publish(std::move(msg));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in timer callback: %s", e.what());
    }
}

} // namespace dexi_cpp

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_cpp::GPIOReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 