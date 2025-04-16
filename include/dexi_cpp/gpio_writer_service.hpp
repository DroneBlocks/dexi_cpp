#ifndef DEXI_CPP__GPIO_WRITER_SERVICE_HPP_
#define DEXI_CPP__GPIO_WRITER_SERVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gpiod.hpp>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include "dexi_interfaces/srv/servo_control.hpp"
#include "dexi_interfaces/srv/gpio_send.hpp"

class GPIOWriterService : public rclcpp::Node
{
public:
    GPIOWriterService();
    ~GPIOWriterService();

private:
    // GPIO pins configuration
    static constexpr int GPIO_PINS[] = {13, 16, 18, 19};  // Output pins
    static constexpr size_t NUM_PINS = sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]);
    
    // ROS2 components
    std::vector<rclcpp::Service<dexi_interfaces::srv::GPIOSend>::SharedPtr> gpio_services_;
    std::vector<rclcpp::Service<dexi_interfaces::srv::ServoControl>::SharedPtr> servo_services_;
    
    // GPIO interface
    std::unique_ptr<gpiod::chip> gpio_chip_;
    std::vector<std::unique_ptr<gpiod::line>> gpio_lines_;
    
    // PWM configuration
    static constexpr int PWM_CHIP = 0;  // PWM chip number
    static constexpr int PWM_PERIOD_NS = 20000000;  // 20ms period (50Hz)
    static constexpr int MIN_DUTY_NS = 500000;      // 0.5ms (0 degrees)
    static constexpr int MAX_DUTY_NS = 2500000;     // 2.5ms (180 degrees)
    
    struct PWMChannel {
        int channel;
        std::string path;
        bool enabled;
    };
    std::vector<PWMChannel> pwm_channels_;
    
    // Callbacks
    void handleGPIORequest(
        const std::shared_ptr<dexi_interfaces::srv::GPIOSend::Request> request,
        std::shared_ptr<dexi_interfaces::srv::GPIOSend::Response> response);
    
    void handleServoRequest(
        const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response);
    
    // Helper functions
    bool initializeGpio();
    void cleanupGpio();
    bool initializePWM();
    void cleanupPWM();
    bool setPWMDutyCycle(const PWMChannel& channel, int duty_ns);
    int mapAngleToPulseWidth(int angle, int min_pw, int max_pw);
    std::string getGPIOServiceName(int pin);
    std::string getServoServiceName(int pin);
    bool writeSysfs(const std::string& path, const std::string& value);
};

#endif // DEXI_CPP__GPIO_WRITER_SERVICE_HPP_ 