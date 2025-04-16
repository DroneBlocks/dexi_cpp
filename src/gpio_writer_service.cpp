#include "gpio_writer_service.hpp"
#include <functional>
#include <filesystem>
#include <fstream>

using std::placeholders::_1;
using std::placeholders::_2;
namespace fs = std::filesystem;

GPIOWriterService::GPIOWriterService()
: Node("gpio_writer_service")
{
    // Initialize GPIO and PWM
    if (!initializeGpio() || !initializePWM()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize GPIO/PWM");
        return;
    }

    // Create services for each GPIO pin
    for (size_t i = 0; i < NUM_PINS; ++i) {
        int pin = GPIO_PINS[i];
        
        // Create GPIO service
        auto gpio_service = create_service<dexi_interfaces::srv::GPIOSend>(
            getGPIOServiceName(pin),
            std::bind(&GPIOWriterService::handleGPIORequest, this, _1, _2)
        );
        gpio_services_.push_back(gpio_service);
        
        // Create servo service
        auto servo_service = create_service<dexi_interfaces::srv::ServoControl>(
            getServoServiceName(pin),
            std::bind(&GPIOWriterService::handleServoRequest, this, _1, _2)
        );
        servo_services_.push_back(servo_service);
        
        RCLCPP_INFO(get_logger(), "Created services for pin %d", pin);
    }
}

GPIOWriterService::~GPIOWriterService()
{
    cleanupGpio();
    cleanupPWM();
}

bool GPIOWriterService::initializeGpio()
{
    try {
        gpio_chip_ = std::make_unique<gpiod::chip>("gpiochip0");
        
        // Initialize GPIO lines for potential digital control
        for (size_t i = 0; i < NUM_PINS; ++i) {
            auto line = gpio_chip_->get_line(GPIO_PINS[i]);
            line.request({"dexi_gpio_writer", gpiod::line_request::DIRECTION_OUTPUT, 0});
            gpio_lines_.push_back(std::make_unique<gpiod::line>(std::move(line)));
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "GPIO initialization failed: %s", e.what());
        return false;
    }
}

bool GPIOWriterService::initializePWM()
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
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "PWM initialization failed: %s", e.what());
        return false;
    }
}

void GPIOWriterService::cleanupGpio()
{
    gpio_lines_.clear();
    gpio_chip_.reset();
}

void GPIOWriterService::cleanupPWM()
{
    for (const auto& channel : pwm_channels_) {
        if (channel.enabled) {
            writeSysfs(channel.path + "/enable", "0");
        }
    }
}

void GPIOWriterService::handleGPIORequest(
    const std::shared_ptr<dexi_interfaces::srv::GPIOSend::Request> request,
    std::shared_ptr<dexi_interfaces::srv::GPIOSend::Response> response)
{
    try {
        // Find the line for the requested pin
        auto it = std::find(std::begin(GPIO_PINS), std::end(GPIO_PINS), request->pin);
        if (it == std::end(GPIO_PINS)) {
            throw std::runtime_error("Invalid GPIO pin");
        }
        
        size_t index = std::distance(std::begin(GPIO_PINS), it);
        gpio_lines_[index]->set_value(request->state ? 1 : 0);

        response->success = true;
        response->message = "Successfully set pin " + std::to_string(request->pin) + 
                          " to " + std::to_string(request->state);
    }
    catch (const std::exception& e) {
        response->success = false;
        response->message = "Error setting pin " + std::to_string(request->pin) + 
                          ": " + e.what();
    }
}

void GPIOWriterService::handleServoRequest(
    const std::shared_ptr<dexi_interfaces::srv::ServoControl::Request> request,
    std::shared_ptr<dexi_interfaces::srv::ServoControl::Response> response)
{
    try {
        // Find the channel for the requested pin
        auto it = std::find(std::begin(GPIO_PINS), std::end(GPIO_PINS), request->pin);
        if (it == std::end(GPIO_PINS)) {
            throw std::runtime_error("Invalid servo pin");
        }
        
        size_t index = std::distance(std::begin(GPIO_PINS), it);
        
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

bool GPIOWriterService::setPWMDutyCycle(const PWMChannel& channel, int duty_ns)
{
    return writeSysfs(channel.path + "/duty_cycle", std::to_string(duty_ns));
}

int GPIOWriterService::mapAngleToPulseWidth(int angle, int min_pw, int max_pw)
{
    // Ensure angle is within bounds
    angle = std::max(0, std::min(180, angle));
    
    // Map angle (0-180) to pulse width (min_pw-max_pw)
    return min_pw + (angle * (max_pw - min_pw) / 180);
}

std::string GPIOWriterService::getGPIOServiceName(int pin)
{
    return "~/write_gpio_" + std::to_string(pin);
}

std::string GPIOWriterService::getServoServiceName(int pin)
{
    return "~/servo_control_" + std::to_string(pin);
}

bool GPIOWriterService::writeSysfs(const std::string& path, const std::string& value)
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
    auto node = std::make_shared<GPIOWriterService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 