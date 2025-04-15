#include "gpio_manager.hpp"
#include <rclcpp/rclcpp.hpp>

GpioManager::GpioManager() : Node("gpio_manager")
{
    // Initialize GPIO interface
    if (!initializeGpio()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO interface");
        return;
    }

    // Create publisher for GPIO state changes
    gpio_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>("gpio_state", 10);

    // Create service for setting GPIO state
    set_gpio_service_ = this->create_service<dexi_interfaces::srv::GPIOSend>(
        "gpio_send",
        std::bind(&GpioManager::setGpioCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create service for setting up GPIO
    setup_gpio_service_ = this->create_service<dexi_interfaces::srv::GPIOSetup>(
        "gpio_setup",
        std::bind(&GpioManager::setupGpioCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "GPIO Manager node initialized successfully");
}

bool GpioManager::initializeGpio()
{
    try {
        // Open the GPIO chip
        chip_ = std::make_unique<gpiod::chip>("gpiochip0");
        
        // Reserve and configure all GPIO lines as outputs initially
        for (size_t i = 0; i < NUM_PINS; ++i) {
            auto line = chip_->get_line(GPIO_PINS[i]);
            line.request({"gpio_manager", gpiod::line_request::DIRECTION_OUTPUT, 0});
            gpio_lines_.push_back(std::make_unique<gpiod::line>(line));
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
        cleanupGpio();
        return false;
    }
}

void GpioManager::cleanupGpio()
{
    gpio_lines_.clear();
    chip_.reset();
}

bool GpioManager::isValidPin(int pin)
{
    for (size_t i = 0; i < NUM_PINS; ++i) {
        if (GPIO_PINS[i] == pin) {
            return true;
        }
    }
    return false;
}

int GpioManager::getLineIndex(int pin)
{
    for (size_t i = 0; i < NUM_PINS; ++i) {
        if (GPIO_PINS[i] == pin) {
            return i;
        }
    }
    return -1;
}

void GpioManager::setGpioCallback(
    const dexi_interfaces::srv::GPIOSend::Request::SharedPtr request,
    dexi_interfaces::srv::GPIOSend::Response::SharedPtr response)
{
    // Validate pin number
    if (!isValidPin(request->pin)) {
        response->success = false;
        response->message = "Invalid pin number. Valid pins are: 13, 16, 18, 19, 20, 21, 22, 23, 24";
        return;
    }

    try {
        // Get the line index and set the value
        int line_index = getLineIndex(request->pin);
        gpio_lines_[line_index]->set_value(request->state ? 1 : 0);

        // Publish state change
        auto msg = std_msgs::msg::Bool();
        msg.data = request->state;
        gpio_state_publisher_->publish(msg);

        response->success = true;
        response->message = "GPIO state set successfully";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set GPIO state: ") + e.what();
    }
}

void GpioManager::setupGpioCallback(
    const dexi_interfaces::srv::GPIOSetup::Request::SharedPtr request,
    dexi_interfaces::srv::GPIOSetup::Response::SharedPtr response)
{
    // Validate pin number
    if (!isValidPin(request->pin)) {
        response->success = false;
        response->message = "Invalid pin number. Valid pins are: 13, 16, 18, 19, 20, 21, 22, 23, 24";
        return;
    }

    try {
        // Get the line index
        int line_index = getLineIndex(request->pin);
        
        // Release the current line
        gpio_lines_[line_index].reset();
        
        // Get a new line with the requested configuration
        auto line = chip_->get_line(GPIO_PINS[line_index]);
        if (request->io == "input") {
            line.request({"gpio_manager", gpiod::line_request::DIRECTION_INPUT, 0});
        } else if (request->io == "output") {
            line.request({"gpio_manager", gpiod::line_request::DIRECTION_OUTPUT, 0});
        } else {
            response->success = false;
            response->message = "Invalid IO mode. Must be 'input' or 'output'";
            return;
        }
        
        // Store the new line
        gpio_lines_[line_index] = std::make_unique<gpiod::line>(line);
        
        response->success = true;
        response->message = "GPIO setup successful";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to setup GPIO: ") + e.what();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpioManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 