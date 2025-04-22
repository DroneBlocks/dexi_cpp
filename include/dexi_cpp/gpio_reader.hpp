#ifndef GPIO_READER_HPP
#define GPIO_READER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <vector>
#include <lgpio.h>

class GPIOReader : public rclcpp::Node
{
public:
    GPIOReader();
    ~GPIOReader();

private:
    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> gpio_publishers_;
    
    // GPIO components
    int gpio_handle_;  // lgpio handle for the chip
    std::vector<int> gpio_pins_;  // List of GPIO pins to read
    
    // Callbacks
    void timerCallback();
    
    // Helper functions
    bool initializeGpio();
    void cleanupGpio();
};

#endif // GPIO_READER_HPP 