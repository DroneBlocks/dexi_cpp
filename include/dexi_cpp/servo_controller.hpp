#ifndef DEXI_CPP_SERVO_CONTROLLER_HPP
#define DEXI_CPP_SERVO_CONTROLLER_HPP

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
    void servoControlCallback(
        const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response);
    bool initializePCA9685();
    void writeByte(uint8_t reg, uint8_t value);
    void setPWM(int channel, int on, int off);

    // I2C file descriptor
    int i2c_fd_;
    
    // PCA9685 constants
    static constexpr uint8_t PCA9685_ADDR = 0x40;
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06;
    
    // Servo parameters
    static constexpr int PWM_FREQUENCY = 50;  // 50Hz standard servo frequency
    static constexpr int MIN_PULSE_WIDTH = 150;  // 0 degrees (0.5ms)
    static constexpr int MAX_PULSE_WIDTH = 600;  // 180 degrees (2.5ms)
    static constexpr int MID_PULSE_WIDTH = 375;  // 90 degrees (1.5ms)
    
    rclcpp::Service<dexi_interfaces::srv::ServoControl>::SharedPtr servo_service_;
};

} // namespace dexi_cpp

#endif // DEXI_CPP_SERVO_CONTROLLER_HPP