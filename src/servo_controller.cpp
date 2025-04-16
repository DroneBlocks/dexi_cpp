#include "dexi_cpp/servo_controller.hpp"
#include <functional>
#include <filesystem>
#include <fstream>

using std::placeholders::_1;
using std::placeholders::_2;
namespace fs = std::filesystem;

ServoController::ServoController()
: Node("servo_controller")
{
    // Initialize PWM
    if (!initializePWM()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PWM");
        return;
    }

    // Create services for each PWM pin
    for (size_t i = 0; i < NUM_PINS; ++i) {
        int pin = PWM_PINS[i];
        auto service = create_service<dexi_interfaces::srv::ServoControl>(
            getServoServiceName(pin),
            std::bind(&ServoController::handleServoRequest, this, _1, _2)
        );
        servo_services_.push_back(service);
        RCLCPP_INFO(get_logger(), "Created servo service for pin %d", pin);
    }
}

ServoController::~ServoController()
{
    cleanupPWM();
}

bool ServoController::initializePWM()
{
    try {
        // Check if PWM chip exists
        std::string pwm_chip_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP);
        if (!fs::exists(pwm_chip_path)) {
            RCLCPP_ERROR(get_logger(), "PWM chip not found at %s", pwm_chip_path.c_str());
            return false;
        }

        // Initialize PWM channels
        for (size_t i = 0; i < NUM_PINS; ++i) {
            PWMChannel channel;
            channel.channel = i;
            channel.path = pwm_chip_path + "/pwm" + std::to_string(i);
            channel.enabled = false;

            // Export PWM channel if not already exported
            if (!fs::exists(channel.path)) {
                writeSysfs(pwm_chip_path + "/export", std::to_string(i));
            }

            // Set period
            writeSysfs(channel.path + "/period", std::to_string(PWM_PERIOD_NS));
            
            // Enable PWM
            writeSysfs(channel.path + "/enable", "1");
            channel.enabled = true;

            pwm_channels_.push_back(channel);
            RCLCPP_INFO(get_logger(), "Initialized PWM channel %d for pin %d", i, PWM_PINS[i]);
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "PWM initialization failed: %s", e.what());
        return false;
    }
}

void ServoController::cleanupPWM()
{
    for (const auto& channel : pwm_channels_) {
        if (channel.enabled) {
            writeSysfs(channel.path + "/enable", "0");
        }
    }
}

void ServoController::handleServoRequest(
    const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
    std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response)
{
    try {
        // Find the channel for the requested pin
        auto it = std::find(std::begin(PWM_PINS), std::end(PWM_PINS), request->pin);
        if (it == std::end(PWM_PINS)) {
            throw std::runtime_error("Invalid servo pin");
        }
        
        size_t index = std::distance(std::begin(PWM_PINS), it);
        
        // Calculate duty cycle based on angle
        int min_pw = request->min_pw > 0 ? request->min_pw : MIN_DUTY_NS / 1000;
        int max_pw = request->max_pw > 0 ? request->max_pw : MAX_DUTY_NS / 1000;
        int pulse_width_ns = mapAngleToPulseWidth(request->angle, min_pw, max_pw) * 1000;
        
        if (!setPWMDutyCycle(pwm_channels_[index], pulse_width_ns)) {
            throw std::runtime_error("Failed to set PWM duty cycle");
        }

        response->success = true;
        response->message = "Successfully set servo " + std::to_string(request->pin) + 
                          " to angle " + std::to_string(request->angle);
    }
    catch (const std::exception& e) {
        response->success = false;
        response->message = "Error setting servo " + std::to_string(request->pin) + 
                          ": " + e.what();
    }
}

bool ServoController::setPWMDutyCycle(const PWMChannel& channel, int duty_ns)
{
    return writeSysfs(channel.path + "/duty_cycle", std::to_string(duty_ns));
}

int ServoController::mapAngleToPulseWidth(int angle, int min_pw, int max_pw)
{
    // Ensure angle is within bounds
    angle = std::max(0, std::min(180, angle));
    
    // Map angle (0-180) to pulse width (min_pw-max_pw)
    return min_pw + (angle * (max_pw - min_pw) / 180);
}

std::string ServoController::getServoServiceName(int pin)
{
    return "~/servo_control_" + std::to_string(pin);
}

bool ServoController::writeSysfs(const std::string& path, const std::string& value)
{
    try {
        std::ofstream ofs(path);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", path.c_str());
            return false;
        }
        ofs << value;
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to write to %s: %s", path.c_str(), e.what());
        return false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 