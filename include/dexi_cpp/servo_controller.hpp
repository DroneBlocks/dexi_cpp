#ifndef DEXI_CPP_SERVO_CONTROLLER_HPP
#define DEXI_CPP_SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "dexi_interfaces/srv/servo_control.hpp"

namespace dexi_cpp
{

class ServoController : public rclcpp::Node
{
public:
    ServoController();
    ~ServoController();

private:
    void servoControlCallback(
        const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response);
    bool initializeServoPins();

    int gpio_handle_;
    rclcpp::Service<dexi_interfaces::srv::ServoControl>::SharedPtr servo_service_;
    std::vector<int> servo_pins_;
    static constexpr int PWM_FREQUENCY = 50;  // 50Hz standard servo frequency
    static constexpr int MIN_PULSE_WIDTH = 500;  // 0.5ms (0 degrees)
    static constexpr int MAX_PULSE_WIDTH = 2500; // 2.5ms (180 degrees)
    static constexpr int MID_PULSE_WIDTH = 1500; // 1.5ms (90 degrees)
};

} // namespace dexi_cpp

#endif // DEXI_CPP_SERVO_CONTROLLER_HPP