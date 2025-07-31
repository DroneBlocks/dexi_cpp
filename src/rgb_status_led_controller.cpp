#include "dexi_cpp/rgb_status_led_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

namespace dexi_cpp
{

RGBStatusLEDController::RGBStatusLEDController()
: Node("rgb_status_led_controller")
{
    // Open I2C bus
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open I2C bus");
        return;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd_, I2C_SLAVE, PCA9685_ADDR) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to set I2C slave address");
        close(i2c_fd_);
        return;
    }

    // Initialize PCA9685
    if (!initializePCA9685()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PCA9685");
        close(i2c_fd_);
        return;
    }

    // Create service for LED control
    led_service_ = create_service<dexi_interfaces::srv::LEDControl>(
        "led_control",
        std::bind(&RGBStatusLEDController::ledControlCallback, this, _1, _2)
    );
    RCLCPP_INFO(get_logger(), "Created service for LED control");

    // Set all LEDs to HIGH by default
    setAllPinsHigh();
    RCLCPP_INFO(get_logger(), "RGB Status LEDs initialized to HIGH state");
}

RGBStatusLEDController::~RGBStatusLEDController()
{
    RCLCPP_INFO(get_logger(), "Shutting down RGB status LED controller");
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

bool RGBStatusLEDController::initializePCA9685()
{
    try {
        // Reset device
        writeByte(MODE1, 0x80);
        usleep(10000); // 10ms delay

        // Set mode1 register (normal mode, auto-increment)
        writeByte(MODE1, 0x20);
        usleep(10000);

        // Set PWM frequency to ~60Hz (good for LED control)
        // Prescale = round(osc_clock / (4096 * update_rate)) - 1
        // For 25MHz clock and 60Hz: prescale = round(25000000 / (4096 * 60)) - 1 = 101
        uint8_t prescale_value = 101;
        writeByte(PRESCALE, prescale_value);
        usleep(10000);

        // Set mode1 register again to enable the new prescale value
        writeByte(MODE1, 0x20);
        usleep(10000);

        RCLCPP_INFO(get_logger(), "Successfully initialized PCA9685");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PCA9685: %s", e.what());
        return false;
    }
}

void RGBStatusLEDController::writeByte(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        throw std::runtime_error("Failed to write to I2C device");
    }
}

void RGBStatusLEDController::setPWM(int channel, int on, int off)
{
    if (channel < 0 || channel > 15) {
        RCLCPP_ERROR(get_logger(), "Invalid channel: %d", channel);
        return;
    }

    uint8_t buffer[5];
    buffer[0] = LED0_ON_L + 4 * channel;
    buffer[1] = on & 0xFF;
    buffer[2] = (on >> 8) & 0xFF;
    buffer[3] = off & 0xFF;
    buffer[4] = (off >> 8) & 0xFF;

    if (write(i2c_fd_, buffer, 5) != 5) {
        RCLCPP_ERROR(get_logger(), "Failed to set PWM for channel %d", channel);
    }
}

void RGBStatusLEDController::setPinHigh(int channel)
{
    // Set pin to fully ON (4095 is maximum PWM value)
    setPWM(channel, 0, 4095);
    RCLCPP_DEBUG(get_logger(), "Set pin %d HIGH", channel);
}

void RGBStatusLEDController::setPinLow(int channel)
{
    // Set pin to fully OFF
    setPWM(channel, 0, 0);
    RCLCPP_DEBUG(get_logger(), "Set pin %d LOW", channel);
}

void RGBStatusLEDController::setAllPinsHigh()
{
    setPinHigh(RED_LED_PIN);
    setPinHigh(GREEN_LED_PIN);
    setPinHigh(BLUE_LED_PIN);
    RCLCPP_INFO(get_logger(), "All RGB LEDs set to HIGH");
}

void RGBStatusLEDController::setAllPinsLow()
{
    setPinLow(RED_LED_PIN);
    setPinLow(GREEN_LED_PIN);
    setPinLow(BLUE_LED_PIN);
    RCLCPP_INFO(get_logger(), "All RGB LEDs set to LOW");
}

void RGBStatusLEDController::ledControlCallback(
    const std::shared_ptr<dexi_interfaces::srv::LEDControl::Request> request,
    std::shared_ptr<dexi_interfaces::srv::LEDControl::Response> response)
{
    try {
        std::string led_name = request->led_name;
        bool state = request->state;
        
        // Convert LED name to pin number
        int pin = -1;
        if (led_name == "red" || led_name == "RED") {
            pin = RED_LED_PIN;
        } else if (led_name == "green" || led_name == "GREEN") {
            pin = GREEN_LED_PIN;
        } else if (led_name == "blue" || led_name == "BLUE") {
            pin = BLUE_LED_PIN;
        } else if (led_name == "all" || led_name == "ALL") {
            // Handle all LEDs
            if (state) {
                setAllPinsHigh();
                response->success = true;
                response->message = "All LEDs set to HIGH";
                RCLCPP_INFO(get_logger(), "All LEDs set to HIGH");
            } else {
                setAllPinsLow();
                response->success = true;
                response->message = "All LEDs set to LOW";
                RCLCPP_INFO(get_logger(), "All LEDs set to LOW");
            }
            return;
        } else {
            response->success = false;
            response->message = "Invalid LED name. Use 'red', 'green', 'blue', or 'all'";
            RCLCPP_ERROR(get_logger(), "Invalid LED name: %s", led_name.c_str());
            return;
        }

        // Set individual LED state
        if (state) {
            setPinHigh(pin);
            response->message = led_name + " LED set to HIGH";
        } else {
            setPinLow(pin);
            response->message = led_name + " LED set to LOW";
        }

        response->success = true;
        RCLCPP_INFO(get_logger(), "%s LED set to %s", led_name.c_str(), state ? "HIGH" : "LOW");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in LED control callback: %s", e.what());
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}

} // namespace dexi_cpp

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_cpp::RGBStatusLEDController>();
    RCLCPP_INFO(node->get_logger(), "RGB Status LED Controller started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 