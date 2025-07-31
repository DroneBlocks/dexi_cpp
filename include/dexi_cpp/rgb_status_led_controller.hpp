#ifndef DEXI_CPP_RGB_STATUS_LED_CONTROLLER_HPP_
#define DEXI_CPP_RGB_STATUS_LED_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "dexi_interfaces/srv/led_control.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <string>

namespace dexi_cpp
{

class RGBStatusLEDController : public rclcpp::Node
{
public:
    RGBStatusLEDController();
    ~RGBStatusLEDController();

private:
    // I2C communication methods
    bool initializePCA9685();
    void writeByte(uint8_t reg, uint8_t value);
    void setPWM(int channel, int on, int off);

    // LED control methods
    void setPinHigh(int channel);
    void setPinLow(int channel);
    void setAllPinsHigh();
    void setAllPinsLow();

    // Service callback
    void ledControlCallback(
        const std::shared_ptr<dexi_interfaces::srv::LEDControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::LEDControl::Response> response);

    // PCA9685 constants
    static constexpr uint8_t PCA9685_ADDR = 0x40;
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06;

    // LED pin assignments
    static constexpr int RED_LED_PIN = 13;
    static constexpr int GREEN_LED_PIN = 14;
    static constexpr int BLUE_LED_PIN = 15;

    // Member variables
    int i2c_fd_;
    rclcpp::Service<dexi_interfaces::srv::LEDControl>::SharedPtr led_service_;
};

} // namespace dexi_cpp

#endif // DEXI_CPP_RGB_STATUS_LED_CONTROLLER_HPP_ 