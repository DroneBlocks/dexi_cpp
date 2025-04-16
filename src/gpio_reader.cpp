#include "dexi_cpp/gpio_reader.hpp"
#include <rclcpp/rclcpp.hpp>

GPIOReader::GPIOReader() : Node("gpio_reader")
{
    // Initialize GPIO interface
    if (!initializeGpio()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO interface");
        return;
    }

    // Create publishers for each GPIO pin
    for (size_t i = 0; i < NUM_PINS; ++i) {
        int pin = GPIO_PINS[i];
        std::string topic_name = getPinTopicName(pin);
        gpio_state_publishers_.push_back(
            this->create_publisher<std_msgs::msg::Bool>(topic_name, 10));
    }

    // Create timer for reading GPIO states
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(250),  // 250ms = 0.25s
        std::bind(&GPIOReader::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "GPIO Reader node initialized successfully");
}

std::string GPIOReader::getPinTopicName(int pin)
{
    return "gpio_" + std::to_string(pin) + "_input";
}

bool GPIOReader::initializeGpio()
{
    try {
        // Open the GPIO chip
        chip_ = std::make_unique<gpiod::chip>("gpiochip0");
        
        // Reserve and configure all GPIO lines as inputs with pull-up
        for (size_t i = 0; i < NUM_PINS; ++i) {
            auto line = chip_->get_line(GPIO_PINS[i]);
            line.request({"gpio_reader", gpiod::line_request::DIRECTION_INPUT, gpiod::line_request::FLAG_BIAS_PULL_UP});
            gpio_lines_.push_back(std::make_unique<gpiod::line>(line));
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
        cleanupGpio();
        return false;
    }
}

void GPIOReader::cleanupGpio()
{
    gpio_lines_.clear();
    chip_.reset();
}

void GPIOReader::timerCallback()
{
    for (size_t i = 0; i < NUM_PINS; ++i) {
        try {
            // Read the current state of the GPIO line
            bool pin_state = gpio_lines_[i]->get_value() != 0;
            
            // Create and publish the message
            auto msg = std_msgs::msg::Bool();
            msg.data = pin_state;
            gpio_state_publishers_[i]->publish(msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read GPIO pin %d: %s", GPIO_PINS[i], e.what());
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIOReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 