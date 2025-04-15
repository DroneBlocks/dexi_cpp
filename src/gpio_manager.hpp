#ifndef GPIO_MANAGER_HPP
#define GPIO_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gpiod.hpp>
#include <memory>
#include <string>
#include <vector>

// Custom service messages
#include "dexi_interfaces/srv/gpio_send.hpp"
#include "dexi_interfaces/srv/gpio_setup.hpp"

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
    rclcpp::Service<dexi_interfaces::srv::GPIOSend>::SharedPtr set_gpio_service_;
    rclcpp::Service<dexi_interfaces::srv::GPIOSetup>::SharedPtr setup_gpio_service_;
    
    // GPIO interface
    std::unique_ptr<gpiod::chip> chip_;
    std::vector<std::unique_ptr<gpiod::line>> gpio_lines_;
    
    // Callbacks
    void setGpioCallback(
        const dexi_interfaces::srv::GPIOSend::Request::SharedPtr request,
        dexi_interfaces::srv::GPIOSend::Response::SharedPtr response);
    
    void setupGpioCallback(
        const dexi_interfaces::srv::GPIOSetup::Request::SharedPtr request,
        dexi_interfaces::srv::GPIOSetup::Response::SharedPtr response);
    
    // Helper functions
    bool initializeGpio();
    void cleanupGpio();
    bool isValidPin(int pin);
    int getLineIndex(int pin);
};

#endif // GPIO_MANAGER_HPP 