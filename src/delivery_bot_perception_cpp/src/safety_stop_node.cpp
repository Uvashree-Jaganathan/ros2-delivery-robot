#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class SafetyStopNode : public rclcpp::Node
{
public:
  SafetyStopNode() : Node("safety_stop_node")
  {
    person_detected_ = false;
    last_cmd_time_ = this->now();
    nearest_obstacle_distance_ = std::numeric_limits<double>::infinity();
    nearest_left_obstacle_distance_ = std::numeric_limits<double>::infinity();
    nearest_right_obstacle_distance_ = std::numeric_limits<double>::infinity();

    stop_distance_ = this->declare_parameter<double>("stop_distance", 0.55);
    slow_distance_ = this->declare_parameter<double>("slow_distance", 1.00);
    scan_forward_angle_ = this->declare_parameter<double>("scan_forward_angle", 0.90);
    caution_speed_scale_ = this->declare_parameter<double>("caution_speed_scale", 0.45);
    recovery_turn_angular_speed_ = this->declare_parameter<double>(
      "recovery_turn_angular_speed",
      0.45
    );
    min_turn_command_ = this->declare_parameter<double>("min_turn_command", 0.05);
    const int cmd_timeout_ms = this->declare_parameter<int>("cmd_timeout_ms", 500);
    cmd_timeout_ = std::chrono::milliseconds(cmd_timeout_ms);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed",
      10,
      std::bind(&SafetyStopNode::cmdCallback, this, std::placeholders::_1)
    );

    detection_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/perception/person_detected",
      10,
      std::bind(&SafetyStopNode::detectionCallback, this, std::placeholders::_1)
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&SafetyStopNode::scanCallback, this, std::placeholders::_1)
    );

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe",
      10
    );

    watchdog_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&SafetyStopNode::watchdogCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Safety stop node started");
  }

private:
  void detectionCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    person_detected_ = msg->data;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double nearest = std::numeric_limits<double>::infinity();
    double nearest_left = std::numeric_limits<double>::infinity();
    double nearest_right = std::numeric_limits<double>::infinity();

    const double angle_min = static_cast<double>(msg->angle_min);
    const double angle_increment = static_cast<double>(msg->angle_increment);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double angle = angle_min + static_cast<double>(i) * angle_increment;
      if (std::abs(angle) > scan_forward_angle_) {
        continue;
      }

      const float range = msg->ranges[i];
      if (
        std::isfinite(range) &&
        range >= msg->range_min &&
        range <= msg->range_max &&
        range < nearest)
      {
        nearest = range;
      }

      if (angle >= 0.0 && range < nearest_left) {
        nearest_left = range;
      } else if (angle < 0.0 && range < nearest_right) {
        nearest_right = range;
      }
    }

    nearest_obstacle_distance_ = nearest;
    nearest_left_obstacle_distance_ = nearest_left;
    nearest_right_obstacle_distance_ = nearest_right;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    geometry_msgs::msg::Twist output;

    if (person_detected_) {
      output = makeStopCommand();

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Person detected. Robot stopped."
      );
    } else if (mustStop()) {
      output = makeAvoidanceCommand(*msg);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Obstacle stop zone at %.2f m. Blocking forward motion and preserving rotation.",
        nearest_obstacle_distance_
      );
    } else if (shouldSlow()) {
      output = *msg;
      output.linear.x *= caution_speed_scale_;
      output.linear.y *= caution_speed_scale_;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Obstacle in caution zone at %.2f m. Scaling linear velocity.",
        nearest_obstacle_distance_
      );
    } else {
      output = *msg;
    }

    cmd_pub_->publish(output);
  }

  void watchdogCallback()
  {
    if ((this->now() - last_cmd_time_) > rclcpp::Duration(cmd_timeout_)) {
      publishStop();
    }
  }

  geometry_msgs::msg::Twist makeStopCommand() const
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.linear.y = 0.0;
    stop.angular.z = 0.0;
    return stop;
  }

  geometry_msgs::msg::Twist makeAvoidanceCommand(
    const geometry_msgs::msg::Twist & input) const
  {
    geometry_msgs::msg::Twist output;
    output.linear.x = 0.0;
    output.linear.y = 0.0;
    output.angular.z = input.angular.z;

    if (std::abs(output.angular.z) < min_turn_command_) {
      output.angular.z = recoveryTurnDirection() * recovery_turn_angular_speed_;
    }

    return output;
  }

  double recoveryTurnDirection() const
  {
    if (nearest_left_obstacle_distance_ < nearest_right_obstacle_distance_) {
      return -1.0;
    }

    return 1.0;
  }

  bool mustStop() const
  {
    return nearest_obstacle_distance_ <= stop_distance_;
  }

  bool shouldSlow() const
  {
    return nearest_obstacle_distance_ <= slow_distance_;
  }

  void publishStop()
  {
    cmd_pub_->publish(makeStopCommand());
  }

  bool person_detected_;
  double nearest_obstacle_distance_;
  double nearest_left_obstacle_distance_;
  double nearest_right_obstacle_distance_;
  double stop_distance_;
  double slow_distance_;
  double scan_forward_angle_;
  double caution_speed_scale_;
  double recovery_turn_angular_speed_;
  double min_turn_command_;
  std::chrono::milliseconds cmd_timeout_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStopNode>());
  rclcpp::shutdown();
  return 0;
}
