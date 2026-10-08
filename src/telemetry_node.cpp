// Low-rate flight telemetry for browsers and Node-RED.
//
// The PX4 topics under /fmu/out arrive at up to ~100 Hz. On-board C++ consumers
// handle that cheaply, but rosbridge converts every message to JSON in Python:
// vehicle_odometry alone cost an ARK CM4 a full core. This node reads those
// topics once and publishes one small JSON summary on /dexi/telemetry at
// publish_rate_hz (default 2), so a dashboard pays for two messages a second.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/estimator_status_flags.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::EstimatorStatusFlags;
using px4_msgs::msg::FailsafeFlags;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace
{

const char * nav_state_name(uint8_t s)
{
  switch (s) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL: return "Manual";
    case VehicleStatus::NAVIGATION_STATE_ALTCTL: return "Altitude";
    case VehicleStatus::NAVIGATION_STATE_POSCTL: return "Position";
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "Mission";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return "Hold";
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return "Return";
    case VehicleStatus::NAVIGATION_STATE_ACRO: return "Acro";
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD: return "Offboard";
    case VehicleStatus::NAVIGATION_STATE_STAB: return "Stabilized";
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "Takeoff";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return "Land";
    default: return "Other";
  }
}

// JSON has no NaN; PX4 uses NaN for "unknown".
std::string num(double v, int decimals)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
  return buf;
}

std::string boolean(bool b) {return b ? "true" : "false";}

double deg(double rad) {return rad * 180.0 / M_PI;}

std::string read_first_line(const char * path)
{
  std::ifstream f(path);
  std::string s;
  std::getline(f, s);
  return s;
}

}  // namespace

class TelemetryNode : public rclcpp::Node
{
public:
  TelemetryNode()
  : Node("telemetry")
  {
    const double rate = declare_parameter("publish_rate_hz", 2.0);

    // PX4's uXRCE-DDS topics are best-effort; a reliable subscription gets nothing.
    const auto qos = rclcpp::SensorDataQoS();
    // Position comes from the offboard manager's 20 Hz copy, which costs a fifth of
    // the raw 100 Hz topic. If that copy goes quiet (offboard disabled), switch to
    // the raw topic until it returns.
    pos_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_20hz", qos,
      [this](VehicleLocalPosition::ConstSharedPtr m) {
        pos_ = m; pos_time_ = now(); pos_20hz_time_ = pos_time_;
      });
    fallback_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {check_position_source();});
    // PX4 1.16 publishes vehicle_status_v1; earlier releases used vehicle_status.
    auto on_status = [this](VehicleStatus::ConstSharedPtr m) {status_ = m; status_time_ = now();};
    status_sub_ = create_subscription<VehicleStatus>("/fmu/out/vehicle_status_v1", qos, on_status);
    status_legacy_sub_ = create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, on_status);
    battery_sub_ = create_subscription<BatteryStatus>(
      "/fmu/out/battery_status", qos,
      [this](BatteryStatus::ConstSharedPtr m) {battery_ = m; battery_time_ = now();});
    est_sub_ = create_subscription<EstimatorStatusFlags>(
      "/fmu/out/estimator_status_flags", qos,
      [this](EstimatorStatusFlags::ConstSharedPtr m) {est_ = m; est_time_ = now();});
    land_sub_ = create_subscription<VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos,
      [this](VehicleLandDetected::ConstSharedPtr m) {land_ = m; land_time_ = now();});
    failsafe_sub_ = create_subscription<FailsafeFlags>(
      "/fmu/out/failsafe_flags", qos,
      [this](FailsafeFlags::ConstSharedPtr m) {failsafe_ = m; failsafe_time_ = now();});

    pub_ = create_publisher<std_msgs::msg::String>("/dexi/telemetry", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
      [this]() {publish();});
  }

private:
  void check_position_source()
  {
    const bool copy_alive = fresh(pos_20hz_time_, 2.0);
    if (!copy_alive && !raw_pos_sub_) {
      raw_pos_sub_ = create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        [this](VehicleLocalPosition::ConstSharedPtr m) {pos_ = m; pos_time_ = now();});
      RCLCPP_INFO(get_logger(), "20 Hz position copy quiet; reading /fmu/out/vehicle_local_position");
    } else if (copy_alive && raw_pos_sub_) {
      raw_pos_sub_.reset();
      RCLCPP_INFO(get_logger(), "20 Hz position copy back; dropped the raw subscription");
    }
  }

  // estimator_status_flags arrives at ~1.4 Hz, so allow a few seconds.
  bool fresh(const rclcpp::Time & t, double max_age = 3.0) const
  {
    return t.nanoseconds() > 0 && (now() - t).seconds() < max_age;
  }

  void publish()
  {
    std::string j = "{";

    if (pos_ && fresh(pos_time_)) {
      const auto & p = *pos_;
      double heading = deg(p.heading);
      if (heading < 0) {
        heading += 360.0;
      }
      j += "\"x\":" + num(p.x, 2) + ",\"y\":" + num(p.y, 2) + ",\"z\":" + num(p.z, 2);
      j += ",\"alt\":" + num(-p.z, 2);
      j += ",\"vx\":" + num(p.vx, 2) + ",\"vy\":" + num(p.vy, 2) + ",\"vz\":" + num(p.vz, 2);
      j += ",\"speed\":" + num(std::hypot(p.vx, p.vy), 2);
      j += ",\"heading\":" + num(heading, 0);
      j += ",\"pos_valid\":" + boolean(p.xy_valid && p.z_valid);
      j += ",\"eph\":" + num(p.eph, 2) + ",\"epv\":" + num(p.epv, 2);
      j += ",\"range_m\":" + num(p.dist_bottom, 2);
      j += ",\"range_valid\":" + boolean(p.dist_bottom_valid) + ",";
    }

    if (status_ && fresh(status_time_)) {
      const auto & s = *status_;
      j += "\"armed\":" + boolean(s.arming_state == VehicleStatus::ARMING_STATE_ARMED);
      j += ",\"nav_state\":" + std::to_string(s.nav_state);
      j += ",\"mode\":\"" + std::string(nav_state_name(s.nav_state)) + "\"";
      j += ",\"failsafe\":" + boolean(s.failsafe);
      j += ",\"preflight_ok\":" + boolean(s.pre_flight_checks_pass) + ",";
    }

    if (battery_ && fresh(battery_time_)) {
      j += "\"battery_pct\":" +
        num(battery_->remaining >= 0 ? battery_->remaining * 100.0 : NAN, 0);
      j += ",\"voltage\":" + num(battery_->voltage_v > 0 ? battery_->voltage_v : NAN, 2) + ",";
    }

    if (est_ && fresh(est_time_)) {
      const auto & e = *est_;
      j += "\"flow_fused\":" + boolean(e.cs_opt_flow);
      j += ",\"flow_rejected\":" + boolean(
        e.reject_optflow_x || e.reject_optflow_y || e.fs_bad_optflow_x || e.fs_bad_optflow_y);
      j += ",\"range_fused\":" + boolean(e.cs_rng_hgt);
      j += ",\"range_fault\":" + boolean(e.cs_rng_fault) + ",";
    }

    if (land_ && fresh(land_time_)) {
      j += "\"landed\":" + boolean(land_->landed) + ",";
    }

    if (failsafe_ && fresh(failsafe_time_)) {
      j += "\"rc_lost\":" + boolean(failsafe_->manual_control_signal_lost);
      j += ",\"low_battery\":" + boolean(
        failsafe_->battery_warning >= BatteryStatus::WARNING_LOW ||
        failsafe_->battery_low_remaining_time) + ",";
    }

    // Companion computer health: plain file reads, no subprocesses.
    const std::string temp = read_first_line("/sys/class/thermal/thermal_zone0/temp");
    const std::string load = read_first_line("/proc/loadavg");
    const std::string thr = read_first_line("/sys/devices/platform/soc/soc:firmware/get_throttled");
    if (!temp.empty()) {
      j += "\"cpu_temp\":" + num(std::stod(temp) / 1000.0, 1) + ",";
    }
    if (!load.empty()) {
      j += "\"cpu_load\":" + num(std::stod(load), 2) + ",";
    }
    if (!thr.empty()) {
      const unsigned long bits = std::stoul(thr, nullptr, 16);
      // Bits 0-3 are "now": undervoltage, frequency capped, throttled, soft temp limit.
      j += "\"throttled\":" + boolean(bits & 0xF) + ",";
    }

    j += "\"connected\":" + boolean(pos_ && fresh(pos_time_)) + "}";

    std_msgs::msg::String out;
    out.data = j;
    pub_->publish(out);
  }

  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr raw_pos_sub_;
  rclcpp::TimerBase::SharedPtr fallback_timer_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_legacy_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<EstimatorStatusFlags>::SharedPtr est_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleLocalPosition::ConstSharedPtr pos_;
  VehicleStatus::ConstSharedPtr status_;
  BatteryStatus::ConstSharedPtr battery_;
  EstimatorStatusFlags::ConstSharedPtr est_;
  VehicleLandDetected::ConstSharedPtr land_;
  FailsafeFlags::ConstSharedPtr failsafe_;
  rclcpp::Time pos_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pos_20hz_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time est_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time land_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time failsafe_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
