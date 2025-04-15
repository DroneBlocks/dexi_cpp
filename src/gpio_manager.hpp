#ifndef GPIO_MANAGER_HPP
#define GPIO_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gpiod.hpp>
#include <memory>
#include <string>
#include <vector>

// Custom service messages
#include "gpio_manager/srv/set_gpio.hpp"
#include "gpio_manager/srv/read_gpio.hpp"

class GpioManager : public rclcpp::Node
{
public:
    GpioManager();

private:
    // GPIO pins configuration
    static constexpr int GPIO_PINS[] = {13, 16, 18, 19, 20, 21, 22, 23, 24};
    static constexpr size_t NUM_PINS = sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]);
    
    // ROS2 components
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gpio_state_publisher_;
    rclcpp::Service<gpio_manager::srv::SetGpio>::SharedPtr set_gpio_service_;
    rclcpp::Service<gpio_manager::srv::ReadGpio>::SharedPtr read_gpio_service_;
    
    // GPIO interface
    std::unique_ptr<gpiod::chip> chip_;
    std::vector<std::unique_ptr<gpiod::line>> gpio_lines_;
    
    // Callbacks
    void setGpioCallback(
        const gpio_manager::srv::SetGpio::Request::SharedPtr request,
        gpio_manager::srv::SetGpio::Response::SharedPtr response);
    
    void readGpioCallback(
        const gpio_manager::srv::ReadGpio::Request::SharedPtr request,
        gpio_manager::srv::ReadGpio::Response::SharedPtr response);
    
    // Helper functions
    bool initializeGpio();
    void cleanupGpio();
    bool isValidPin(int pin);
    int getLineIndex(int pin);
};

#endif // GPIO_MANAGER_HPP 