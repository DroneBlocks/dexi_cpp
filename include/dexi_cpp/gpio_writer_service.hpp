#ifndef DEXI_CPP_GPIO_WRITER_SERVICE_HPP
#define DEXI_CPP_GPIO_WRITER_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include "dexi_interfaces/srv/set_gpio.hpp"
#include <vector>
#include <map>

namespace dexi_cpp
{

class GPIOWriterService : public rclcpp::Node
{
public:
    GPIOWriterService();
    ~GPIOWriterService();

private:
    void handleWriteRequest(
        const std::shared_ptr<dexi_interfaces::srv::SetGpio::Request> request,
        std::shared_ptr<dexi_interfaces::srv::SetGpio::Response> response);
    bool initializeGpio();
    void cleanupGpio();

    int gpio_handle_;
    std::vector<int> gpio_pins_;
    rclcpp::Service<dexi_interfaces::srv::SetGpio>::SharedPtr service_;
};

} // namespace dexi_cpp

#endif // DEXI_CPP_GPIO_WRITER_SERVICE_HPP 