#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>
#include <rclcpp/timer.hpp>

class PX4OffboardManager : public rclcpp::Node
{
public:
    PX4OffboardManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PX4OffboardManager();

private:
    // ... existing private member variables ...
    rclcpp::TimerBase::SharedPtr offboard_timer_;
};