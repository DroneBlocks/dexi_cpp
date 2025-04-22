#ifndef DEXI_CPP__GPIO_WRITER_SERVICE_HPP_
#define DEXI_CPP__GPIO_WRITER_SERVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dexi_interfaces/srv/set_gpio.hpp>
#include <lgpio.h>
#include <vector>
#include <memory>

class GPIOWriterService : public rclcpp::Node
{
public:
    GPIOWriterService();

private:
    void handleWriteRequest(
        const std::shared_ptr<dexi_interfaces::srv::SetGpio::Request> request,
        std::shared_ptr<dexi_interfaces::srv::SetGpio::Response> response
    );

    bool initializeGpio();
    void cleanupGpio();

    rclcpp::Service<dexi_interfaces::srv::SetGpio>::SharedPtr service_;
    int gpio_handle_;
    std::vector<int> gpio_pins_;
};

#endif  // DEXI_CPP__GPIO_WRITER_SERVICE_HPP_ 