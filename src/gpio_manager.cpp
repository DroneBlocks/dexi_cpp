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
    set_gpio_service_ = this->create_service<gpio_manager::srv::SetGpio>(
        "set_gpio",
        std::bind(&GpioManager::setGpioCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create service for reading GPIO state
    read_gpio_service_ = this->create_service<gpio_manager::srv::ReadGpio>(
        "read_gpio",
        std::bind(&GpioManager::readGpioCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "GPIO Manager node initialized successfully");
}

bool GpioManager::initializeGpio()
{
    try {
        // Open the GPIO chip
        chip_ = std::make_unique<gpiod::chip>("gpiochip0");
        
        // Reserve and configure all GPIO lines
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
    const gpio_manager::srv::SetGpio::Request::SharedPtr request,
    gpio_manager::srv::SetGpio::Response::SharedPtr response)
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
        gpio_lines_[line_index]->set_value(request->value ? 1 : 0);

        // Publish state change
        auto msg = std_msgs::msg::Bool();
        msg.data = request->value;
        gpio_state_publisher_->publish(msg);

        response->success = true;
        response->message = "GPIO state set successfully";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set GPIO state: ") + e.what();
    }
}

void GpioManager::readGpioCallback(
    const gpio_manager::srv::ReadGpio::Request::SharedPtr request,
    gpio_manager::srv::ReadGpio::Response::SharedPtr response)
{
    // Validate pin number
    if (!isValidPin(request->pin)) {
        response->success = false;
        response->message = "Invalid pin number. Valid pins are: 13, 16, 18, 19, 20, 21, 22, 23, 24";
        return;
    }

    try {
        // Get the line index and read the value
        int line_index = getLineIndex(request->pin);
        response->value = (gpio_lines_[line_index]->get_value() == 1);
        response->success = true;
        response->message = "GPIO state read successfully";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to read GPIO state: ") + e.what();
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