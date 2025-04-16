#ifndef DEXI_CPP__GPIO_READER_HPP_
#define DEXI_CPP__GPIO_READER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gpiod.hpp>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs/msg/bool.hpp"

class GPIOReader : public rclcpp::Node
{
public:
    GPIOReader();

private:
    // GPIO pins configuration for inputs
    static constexpr int GPIO_PINS[] = {20, 21, 22, 23, 24};  // Input pins
    static constexpr size_t NUM_PINS = sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]);
    
    // ROS2 components
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> gpio_state_publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // GPIO interface
    std::unique_ptr<gpiod::chip> chip_;
    std::vector<std::unique_ptr<gpiod::line>> gpio_lines_;
    
    // Callbacks
    void timerCallback();
    
    // Helper functions
    bool initializeGpio();
    void cleanupGpio();
    std::string getPinTopicName(int pin);
};

#endif // DEXI_CPP__GPIO_READER_HPP_ 