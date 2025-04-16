#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <dexi_interfaces/srv/servo_control.hpp>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

class ServoController : public rclcpp::Node
{
public:
    ServoController();
    ~ServoController();

private:
    // PWM configuration
    static constexpr int PWM_CHIP = 0;  // PWM chip number
    static constexpr int PWM_PERIOD_NS = 20000000;  // 20ms period (50Hz)
    static constexpr int MIN_DUTY_NS = 500000;      // 0.5ms (0 degrees)
    static constexpr int MAX_DUTY_NS = 2500000;     // 2.5ms (180 degrees)
    
    // Supported GPIO pins for PWM
    static constexpr int PWM_PINS[] = {13, 16, 18, 19};
    static constexpr size_t NUM_PINS = sizeof(PWM_PINS) / sizeof(PWM_PINS[0]);

    // PWM channel structure
    struct PWMChannel {
        int channel;
        std::string path;
        bool enabled;
    };

    // ROS2 components
    std::vector<rclcpp::Service<dexi_interfaces::srv::ServoControl>::SharedPtr> servo_services_;
    
    // PWM channels
    std::vector<PWMChannel> pwm_channels_;
    
    // Callbacks
    void handleServoRequest(
        const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response);
    
    // Helper functions
    bool initializePWM();
    void cleanupPWM();
    bool setPWMDutyCycle(const PWMChannel& channel, int duty_ns);
    int mapAngleToPulseWidth(int angle, int min_pw, int max_pw);
    std::string getServoServiceName(int pin);
    bool writeSysfs(const std::string& path, const std::string& value);
};

#endif // SERVO_CONTROLLER_HPP