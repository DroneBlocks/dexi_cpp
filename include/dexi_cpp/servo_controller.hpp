#ifndef DEXI_CPP_SERVO_CONTROLLER_HPP_
#define DEXI_CPP_SERVO_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "dexi_interfaces/srv/servo_control.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

namespace dexi_cpp
{

class ServoController : public rclcpp::Node
{
public:
    ServoController();
    ~ServoController();

private:
    // I2C communication methods
    bool initializePCA9685();
    void writeByte(uint8_t reg, uint8_t value);
    void setPWM(int channel, int on, int off);

    // Service callback
    void servoControlCallback(
        const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response);

    // PCA9685 constants
    static constexpr uint8_t PCA9685_ADDR = 0x40;
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06;

    // Member variables
    int i2c_fd_;
    rclcpp::Service<dexi_interfaces::srv::ServoControl>::SharedPtr servo_service_;
};

} // namespace dexi_cpp

#endif // DEXI_CPP_SERVO_CONTROLLER_HPP_