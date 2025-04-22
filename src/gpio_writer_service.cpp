#include "dexi_cpp/gpio_writer_service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;

// GPIO pins configuration for outputs
static constexpr int GPIO_PINS[] = {13, 16, 18, 19};  // Output pins
static constexpr size_t NUM_PINS = sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]);

GPIOWriterService::GPIOWriterService()
: Node("gpio_writer_service")
{
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

    // Create service
    service_ = create_service<dexi_interfaces::srv::SetGpio>(
        "~/set_gpio",
        std::bind(&GPIOWriterService::handleWriteRequest, this, _1, _2)
    );
    RCLCPP_INFO(get_logger(), "GPIO writer service started");
}

bool GPIOWriterService::initializeGpio()
{
    try {
        // Initialize GPIO pins
        for (size_t i = 0; i < NUM_PINS; ++i) {
            int pin = GPIO_PINS[i];
            
            // Claim GPIO for output
            if (lgGpioClaimOutput(gpio_handle_, 0, pin, 0) != LG_OKAY) {
                RCLCPP_ERROR(get_logger(), "Failed to claim GPIO %d for output", pin);
                continue;
            }
            
            gpio_pins_.push_back(pin);
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

void GPIOWriterService::cleanupGpio()
{
    RCLCPP_INFO(get_logger(), "Cleaning up GPIO pins");
    // Release all GPIO pins
    for (int pin : gpio_pins_) {
        lgGpioFree(gpio_handle_, pin);
    }
    gpio_pins_.clear();
    RCLCPP_INFO(get_logger(), "GPIO cleanup complete");
}

void GPIOWriterService::handleWriteRequest(
    const std::shared_ptr<dexi_interfaces::srv::SetGpio::Request> request,
    std::shared_ptr<dexi_interfaces::srv::SetGpio::Response> response)
{
    try {
        // Find the pin in our list
        auto it = std::find(gpio_pins_.begin(), gpio_pins_.end(), request->pin);
        if (it == gpio_pins_.end()) {
            response->success = false;
            response->message = "Pin not configured for output";
            return;
        }

        // Set GPIO value
        if (lgGpioWrite(gpio_handle_, request->pin, request->value ? 1 : 0) != LG_OKAY) {
            response->success = false;
            response->message = "Failed to write GPIO value";
            return;
        }

        response->success = true;
        response->message = "Successfully wrote GPIO value";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIOWriterService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 