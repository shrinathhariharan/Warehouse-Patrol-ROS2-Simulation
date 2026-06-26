#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SafetyFilter : public rclcpp::Node
{
public:
  SafetyFilter()
  : Node("safety_filter")
  {
    stop_distance_ = this->declare_parameter("stop_distance", 0.35);
    slow_distance_ = this->declare_parameter("slow_distance", 0.75);
    front_half_angle_ = this->declare_parameter("front_half_angle", 0.785398);  // 45 deg
    publish_rate_hz_ = this->declare_parameter("publish_rate_hz", 50.0);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_input_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_input", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        desired_cmd_ = *msg;
        have_desired_cmd_ = true;
      });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        updateFrontClearance(*msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->publishFilteredCommand(); });

    RCLCPP_INFO(
      this->get_logger(),
      "Safety filter active (stop=%.2f m, slow=%.2f m, front cone=%.0f deg)",
      stop_distance_, slow_distance_, front_half_angle_ * 180.0 / M_PI);
  }

private:
  void updateFrontClearance(const sensor_msgs::msg::LaserScan & scan)
  {
    double min_range = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (std::abs(angle) > front_half_angle_) {
        continue;
      }

      const float range = scan.ranges[i];
      if (!std::isfinite(range)) {
        continue;
      }
      if (range < scan.range_min || range > scan.range_max) {
        continue;
      }

      min_range = std::min(min_range, static_cast<double>(range));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (std::isfinite(min_range)) {
      front_clearance_ = min_range;
    }
  }

  void publishFilteredCommand()
  {
    geometry_msgs::msg::Twist output;
    bool have_cmd = false;
    double clearance = std::numeric_limits<double>::infinity();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_desired_cmd_) {
        output = desired_cmd_;
        have_cmd = true;
      }
      clearance = front_clearance_;
    }

    if (!have_cmd) {
      return;
    }

    if (clearance <= stop_distance_) {
      output.linear.x = 0.0;
      output.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Emergency stop: obstacle at %.2f m", clearance);
    } else if (clearance <= slow_distance_) {
      const double scale = (clearance - stop_distance_) / (slow_distance_ - stop_distance_);
      output.linear.x *= std::clamp(scale, 0.0, 1.0);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Slowing down: obstacle at %.2f m (scale %.2f)", clearance, scale);
    }

    cmd_vel_pub_->publish(output);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist desired_cmd_;
  bool have_desired_cmd_{false};
  double front_clearance_{std::numeric_limits<double>::infinity()};

  double stop_distance_;
  double slow_distance_;
  double front_half_angle_;
  double publish_rate_hz_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyFilter>());
  rclcpp::shutdown();
  return 0;
}
