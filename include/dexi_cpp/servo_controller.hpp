#ifndef DEXI_CPP_SERVO_CONTROLLER_HPP
#define DEXI_CPP_SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "dexi_interfaces/srv/servo_control.hpp"
#include "dexi_cpp/pca9685.hpp"
#include <memory>
#include <string>

namespace dexi_cpp {

class ServoController : public rclcpp::Node {
public:
    ServoController();
    ~ServoController();

private:
    void servoControlCallback(
        const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response);

    // PCA9685 device instance
    std::unique_ptr<Pca9685> pca9685_dev_;

    // ROS2 service
    rclcpp::Service<dexi_interfaces::srv::ServoControl>::SharedPtr servo_service_;

    // Default servo parameters (can be overridden by service request)
    static constexpr double DEFAULT_SERVO_FREQ = 50.0;      // 50Hz standard servo frequency
    static constexpr double DEFAULT_MIN_PW_US = 500.0;      // 0.5ms minimum pulse width
    static constexpr double DEFAULT_MAX_PW_US = 2500.0;     // 2.5ms maximum pulse width
    static constexpr double PCA9685_OSC_HZ = 25000000.0;    // 25MHz internal oscillator
    static constexpr uint8_t MAX_CHANNEL_NUM = 15U;         // PCA9685 supports 16 channels (0-15)
    static constexpr double MAX_SERVO_ANGLE = 180.0;        // Maximum servo angle in degrees

    // Helper functions
    double calculateDutyCycle(double angle, double min_pw_us, double max_pw_us) const;
    bool isValidChannel(int32_t pin) const;
    bool isValidAngle(int32_t angle) const;
    bool initializePca9685();
};

} // namespace dexi_cpp

#endif  // DEXI_CPP_SERVO_CONTROLLER_HPP