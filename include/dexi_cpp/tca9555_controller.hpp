#ifndef DEXI_CPP_TCA9555_CONTROLLER_HPP
#define DEXI_CPP_TCA9555_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "dexi_interfaces/srv/gpio_send.hpp"

#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <map>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

namespace dexi_cpp
{

class TCA9555Controller : public rclcpp::Node
{
public:
    TCA9555Controller();
    ~TCA9555Controller();

private:
    void handleWriteRequest(
        const std::shared_ptr<dexi_interfaces::srv::GPIOSend::Request> request,
        std::shared_ptr<dexi_interfaces::srv::GPIOSend::Response> response);
    
    void inputPollingTimer();
    void publishPinStates();
    
    bool initializeI2C();
    void cleanupI2C();
    bool writePin(uint8_t pin, bool value);
    bool readPin(uint8_t pin, bool& value);

    bool writeRegister(uint8_t reg, uint16_t value);
    bool readRegister(uint8_t reg, uint16_t& value);
    void updateConfiguration();

    // TCA9555 registers
    static constexpr uint8_t INPUT_PORT_0 = 0x00;
    static constexpr uint8_t INPUT_PORT_1 = 0x01;
    static constexpr uint8_t OUTPUT_PORT_0 = 0x02;
    static constexpr uint8_t OUTPUT_PORT_1 = 0x03;
    static constexpr uint8_t CONFIG_PORT_0 = 0x06;
    static constexpr uint8_t CONFIG_PORT_1 = 0x07;

    int i2c_fd_;
    uint8_t i2c_address_;
    std::string i2c_device_;
    std::vector<uint8_t> available_pins_;
    uint16_t output_state_;
    uint16_t config_state_;  // 0 = output, 1 = input
    std::map<uint8_t, bool> pin_states_;  // Current state of all pins
    std::map<uint8_t, bool> pin_modes_;   // true = output, false = input
    
    rclcpp::Service<dexi_interfaces::srv::GPIOSend>::SharedPtr write_service_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pin_state_publishers_[16];
    rclcpp::TimerBase::SharedPtr input_timer_;
    
    double input_polling_rate_;  // Hz
};

} // namespace dexi_cpp

#endif // DEXI_CPP_TCA9555_CONTROLLER_HPP 