#include "dexi_cpp/tca9555_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <algorithm>
#include <cstring>

using std::placeholders::_1;
using std::placeholders::_2;

namespace dexi_cpp
{

TCA9555Controller::TCA9555Controller()
: Node("tca9555_controller")
{
    // Declare and get parameters
    this->declare_parameter("i2c_device", "/dev/i2c-1");  // Default I2C device
    this->declare_parameter("i2c_address", 0x20);         // TCA9555 address
    this->declare_parameter("available_pins", std::vector<int64_t>{0, 1, 2, 3, 4, 14, 15});  // Pins 0-4, 14-15
    this->declare_parameter("input_polling_rate", 10.0);  // Hz
    // Declare individual pin mode parameters
    this->declare_parameter("pin_0_mode", true);   // Pin 0: output (true=output, false=input)
    this->declare_parameter("pin_1_mode", true);   // Pin 1: output
    this->declare_parameter("pin_2_mode", true);   // Pin 2: output
    this->declare_parameter("pin_3_mode", false);  // Pin 3: input
    this->declare_parameter("pin_4_mode", false);  // Pin 4: input
    this->declare_parameter("pin_14_mode", true);  // Pin 14: output
    this->declare_parameter("pin_15_mode", true);  // Pin 15: output
    
    i2c_device_ = this->get_parameter("i2c_device").as_string();
    i2c_address_ = static_cast<uint8_t>(this->get_parameter("i2c_address").as_int());
    input_polling_rate_ = this->get_parameter("input_polling_rate").as_double();
    
    auto pins_param = this->get_parameter("available_pins").as_integer_array();
    for (const auto& pin : pins_param) {
        available_pins_.push_back(static_cast<uint8_t>(pin));
        pin_modes_[static_cast<uint8_t>(pin)] = true;  // Default to output
    }

    // Load pin modes from individual parameters
    pin_modes_[0] = this->get_parameter("pin_0_mode").as_bool();
    pin_modes_[1] = this->get_parameter("pin_1_mode").as_bool();
    pin_modes_[2] = this->get_parameter("pin_2_mode").as_bool();
    pin_modes_[3] = this->get_parameter("pin_3_mode").as_bool();
    pin_modes_[4] = this->get_parameter("pin_4_mode").as_bool();
    pin_modes_[14] = this->get_parameter("pin_14_mode").as_bool();
    pin_modes_[15] = this->get_parameter("pin_15_mode").as_bool();
    
    // Log pin configurations
    for (uint8_t pin : {0, 1, 2, 3, 4, 14, 15}) {
        RCLCPP_INFO(get_logger(), "Pin %d configured as: %s", pin, pin_modes_[pin] ? "output" : "input");
    }

    // Initialize I2C
    if (!initializeI2C()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize I2C for TCA9555");
        return;
    }

    // Create service for setting pin values
    write_service_ = create_service<dexi_interfaces::srv::SetGpio>(
        "~/set_tca9555_pin",
        std::bind(&TCA9555Controller::handleWriteRequest, this, _1, _2)
    );

    // Create publishers for available pins (0-4, 14-15)
    for (uint8_t pin : {0, 1, 2, 3, 4, 14, 15}) {
        std::string topic_name = "~/gpio_" + std::to_string(pin) + "_state";
        pin_state_publishers_[pin] = create_publisher<std_msgs::msg::Bool>(topic_name, 10);
    }

    // Create input polling timer
    input_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / input_polling_rate_),
        std::bind(&TCA9555Controller::inputPollingTimer, this)
    );

    RCLCPP_INFO(get_logger(), "TCA9555 controller service started on %s at address 0x%02X", 
                i2c_device_.c_str(), i2c_address_);
    RCLCPP_INFO(get_logger(), "Input polling rate: %.1f Hz", input_polling_rate_);
}

TCA9555Controller::~TCA9555Controller()
{
    RCLCPP_INFO(get_logger(), "Shutting down TCA9555 controller");
    cleanupI2C();
}

bool TCA9555Controller::initializeI2C()
{
    try {
        // Open I2C device
        i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open I2C device %s: %s", 
                        i2c_device_.c_str(), strerror(errno));
            return false;
        }

        // Set I2C slave address
        if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to set I2C slave address 0x%02X: %s", 
                        i2c_address_, strerror(errno));
            close(i2c_fd_);
            return false;
        }

        // Initialize configuration and output states
        config_state_ = 0x0000;  // All pins as outputs initially
        output_state_ = 0x0000;  // All outputs low initially

        // Configure pins based on pin_modes_
        updateConfiguration();

        RCLCPP_INFO(get_logger(), "Successfully initialized TCA9555 with %zu pins", available_pins_.size());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "I2C initialization failed: %s", e.what());
        return false;
    }
}

void TCA9555Controller::updateConfiguration()
{
    // Update config_state_ based on pin_modes_
    config_state_ = 0x0000;
    for (const auto& [pin, is_output] : pin_modes_) {
        if (!is_output) {
            config_state_ |= (1 << pin);  // Set as input
        }
    }

    // Write configuration to TCA9555
    if (!writeRegister(CONFIG_PORT_0, config_state_ & 0xFF)) {
        RCLCPP_ERROR(get_logger(), "Failed to configure port 0");
        return;
    }
    
    if (!writeRegister(CONFIG_PORT_1, (config_state_ >> 8) & 0xFF)) {
        RCLCPP_ERROR(get_logger(), "Failed to configure port 1");
        return;
    }

    // Write output state for output pins
    if (!writeRegister(OUTPUT_PORT_0, output_state_ & 0xFF)) {
        RCLCPP_ERROR(get_logger(), "Failed to write port 0 outputs");
        return;
    }
    
    if (!writeRegister(OUTPUT_PORT_1, (output_state_ >> 8) & 0xFF)) {
        RCLCPP_ERROR(get_logger(), "Failed to write port 1 outputs");
        return;
    }

    RCLCPP_INFO(get_logger(), "Updated TCA9555 configuration: 0x%04X", config_state_);
}

void TCA9555Controller::cleanupI2C()
{
    RCLCPP_INFO(get_logger(), "Cleaning up I2C connection");
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
    RCLCPP_INFO(get_logger(), "I2C cleanup complete");
}

bool TCA9555Controller::writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buffer[2] = {reg, static_cast<uint8_t>(value & 0xFF)};
    
    if (write(i2c_fd_, buffer, 2) != 2) {
        RCLCPP_ERROR(get_logger(), "Failed to write register 0x%02X: %s", reg, strerror(errno));
        return false;
    }
    
    return true;
}

bool TCA9555Controller::readRegister(uint8_t reg, uint16_t& value)
{
    uint8_t buffer[1] = {reg};
    
    // Write register address
    if (write(i2c_fd_, buffer, 1) != 1) {
        RCLCPP_ERROR(get_logger(), "Failed to write register address 0x%02X: %s", reg, strerror(errno));
        return false;
    }
    
    // Read register value
    if (read(i2c_fd_, buffer, 1) != 1) {
        RCLCPP_ERROR(get_logger(), "Failed to read register 0x%02X: %s", reg, strerror(errno));
        return false;
    }
    
    value = buffer[0];
    return true;
}

bool TCA9555Controller::writePin(uint8_t pin, bool value)
{
    if (pin >= 5 && pin != 14 && pin != 15) {
        RCLCPP_ERROR(get_logger(), "Invalid pin number: %d (must be 0-4, 14, or 15)", pin);
        return false;
    }

    // Check if pin is configured as output
    if (!pin_modes_[pin]) {
        RCLCPP_ERROR(get_logger(), "Pin %d is configured as input, cannot write", pin);
        return false;
    }

    // Update internal state
    if (value) {
        output_state_ |= (1 << pin);
    } else {
        output_state_ &= ~(1 << pin);
    }

    // Write to appropriate port
    if (pin < 8) {
        // Port 0 (pins 0-7)
        return writeRegister(OUTPUT_PORT_0, output_state_ & 0xFF);
    } else if (pin < 16) {
        // Port 1 (pins 8-15)
        return writeRegister(OUTPUT_PORT_1, (output_state_ >> 8) & 0xFF);
    } else {
        // Pins 14-15 are on Port 1 (pins 8-15)
        // Update internal state
        if (value) {
            output_state_ |= (1 << pin);
        } else {
            output_state_ &= ~(1 << pin);
        }
        
        // Write to Port 1 (pins 8-15)
        return writeRegister(OUTPUT_PORT_1, (output_state_ >> 8) & 0xFF);
    }
}

bool TCA9555Controller::readPin(uint8_t pin, bool& value)
{
    if (pin >= 5 && pin != 14 && pin != 15) {
        RCLCPP_ERROR(get_logger(), "Invalid pin number: %d (must be 0-4, 14, or 15)", pin);
        return false;
    }

    uint16_t input_state;
    if (pin < 8) {
        // Read Port 0 (pins 0-7)
        if (!readRegister(INPUT_PORT_0, input_state)) {
            return false;
        }
    } else if (pin < 16) {
        // Read Port 1 (pins 8-15)
        if (!readRegister(INPUT_PORT_1, input_state)) {
            return false;
        }
    } else {
        // Pins 14-15 are on Port 1 (pins 8-15)
        // Read from Port 1 (pins 8-15)
        if (!readRegister(INPUT_PORT_1, input_state)) {
            return false;
        }
        
        value = (input_state & (1 << (pin - 8))) != 0;
        return true;
    }

    value = (input_state & (1 << (pin % 8))) != 0;
    return true;
}



void TCA9555Controller::inputPollingTimer()
{
    publishPinStates();
}

void TCA9555Controller::publishPinStates()
{
    // Read and publish states for input pins
    for (const auto& [pin, is_output] : pin_modes_) {
        if (!is_output) {  // Only read input pins
            bool value;
            if (readPin(pin, value)) {
                // Update stored state
                pin_states_[pin] = value;
                
                // Always publish current state
                auto msg = std_msgs::msg::Bool();
                msg.data = value;
                pin_state_publishers_[pin]->publish(msg);
                
                RCLCPP_DEBUG(get_logger(), "Pin %d state: %s", pin, value ? "HIGH" : "LOW");
            }
        }
    }
}

void TCA9555Controller::handleWriteRequest(
    const std::shared_ptr<dexi_interfaces::srv::SetGpio::Request> request,
    std::shared_ptr<dexi_interfaces::srv::SetGpio::Response> response)
{
    try {
        // Check if pin is valid (0-4, 14-15)
        if (request->pin >= 5 && request->pin != 14 && request->pin != 15) {
            response->success = false;
            response->message = "Invalid pin number (must be 0-4, 14, or 15)";
            return;
        }

        // Write pin value
        if (!writePin(request->pin, request->value)) {
            response->success = false;
            response->message = "Failed to write TCA9555 pin value";
            return;
        }

        response->success = true;
        response->message = "Successfully wrote TCA9555 pin value";
        RCLCPP_INFO(get_logger(), "Set TCA9555 pin %d to %s", request->pin, request->value ? "HIGH" : "LOW");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}



} // namespace dexi_cpp

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_cpp::TCA9555Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 