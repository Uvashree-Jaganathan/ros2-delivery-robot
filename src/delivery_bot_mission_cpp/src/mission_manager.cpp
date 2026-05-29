#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionManager() : Node("mission_manager")
  {
    goal_completion_tolerance_ = this->declare_parameter<double>(
      "goal_completion_tolerance",
      0.30
    );
    loose_waypoint_tolerance_ = this->declare_parameter<double>(
      "loose_waypoint_tolerance",
      0.90
    );
    strict_goal_tolerance_ = this->declare_parameter<double>(
      "strict_goal_tolerance",
      0.30
    );

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose"
    );

    timer_ = this->create_wall_timer(
      3s,
      std::bind(&MissionManager::startMission, this)
    );

    RCLCPP_INFO(this->get_logger(), "Mission manager started");
  }

private:
  enum class MissionState
  {
    Idle,
    GoingToPickup,
    PickingUpPackage,
    GoingToDropoff,
    DeliveringPackage,
    ReturningToDock,
    MissionComplete,
    Failed
  };

  void startMission()
  {
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 NavigateToPose server");

    if (!nav_client_->wait_for_action_server(30s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server unavailable");
      setState(MissionState::Failed);
      shutdown_timer_ = this->create_wall_timer(
        100ms,
        []() {rclcpp::shutdown();}
      );
      return;
    }

    setState(MissionState::GoingToPickup);
    sendGoal("pickup_zone", -2.5, 1.0, 0.0);
  }

  void sendGoal(
    const std::string & goal_name,
    double x,
    double y,
    double yaw)
  {
    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    active_goal_pose_ = goal_msg.pose.pose;

    auto options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    active_goal_has_moved_ = false;

    options.goal_response_callback =
      [this, goal_name](const GoalHandleNav::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Goal rejected: %s",
            goal_name.c_str()
          );
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "Goal accepted: %s",
            goal_name.c_str()
          );
        }
      };

    options.feedback_callback =
      [this, goal_name](
        GoalHandleNav::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Going to %s. Distance remaining: %.2f",
          goal_name.c_str(),
          feedback->distance_remaining
        );

        const double goal_distance =
          distanceToGoal(feedback->current_pose.pose, active_goal_pose_);

        if (goal_distance > completionTolerance(goal_name) + 0.20) {
          active_goal_has_moved_ = true;
        }

        if (
          active_goal_has_moved_ &&
          goal_distance <= completionTolerance(goal_name) &&
          completed_by_feedback_goals_.count(goal_name) == 0)
        {
          completed_by_feedback_goals_.insert(goal_name);
          RCLCPP_INFO(
            this->get_logger(),
            "Reached %s within %.2f m acceptance radius",
            goal_name.c_str(),
            completionTolerance(goal_name)
          );
          nav_client_->async_cancel_goal(goal_handle);
          handleGoalReached(goal_name);
          return;
        }

        (void)goal_handle;
      };

    options.result_callback =
      [this, goal_name](const GoalHandleNav::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(
            this->get_logger(),
            "Reached %s",
            goal_name.c_str()
          );
          handleGoalReached(goal_name);
        } else if (completed_by_feedback_goals_.count(goal_name) == 0) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to reach %s",
            goal_name.c_str()
          );
          setState(MissionState::Failed);
          rclcpp::shutdown();
        }
      };

    RCLCPP_INFO(
      this->get_logger(),
      "Sending goal: %s (x=%.2f, y=%.2f, yaw=%.2f)",
      goal_name.c_str(),
      x,
      y,
      yaw
    );

    nav_client_->async_send_goal(goal_msg, options);
  }

  void handleGoalReached(const std::string & goal_name)
  {
    if (goal_name == "pickup_zone") {
      setState(MissionState::PickingUpPackage);

      RCLCPP_INFO(this->get_logger(), "Simulating package pickup");
      scheduleNextStep(3s, [this]() {
        setState(MissionState::GoingToDropoff);
        sendGoal("pickup_south_lane", -2.5, -3.4, 0.0);
      });
    }

    else if (goal_name == "pickup_south_lane") {
      sendGoal("delivery_approach", -1.0, -3.4, 0.0);
    }

    else if (goal_name == "delivery_approach") {
      sendGoal("delivery_lane_mid", 1.6, -3.6, 0.0);
    }

    else if (goal_name == "delivery_lane_mid") {
      sendGoal("delivery_zone", 3.3, -2.8, 0.0);
    }

    else if (goal_name == "delivery_zone") {
      setState(MissionState::DeliveringPackage);

      RCLCPP_INFO(this->get_logger(), "Package delivered");
      scheduleNextStep(2s, [this]() {
        setState(MissionState::ReturningToDock);
        sendGoal("return_south_lane", 3.2, -3.8, 3.14);
      });
    }

    else if (goal_name == "return_south_lane") {
      sendGoal("return_mid_lane", 0.0, -3.8, 3.14);
    }

    else if (goal_name == "return_mid_lane") {
      sendGoal("return_west_lane", -2.6, -3.0, 1.57);
    }

    else if (goal_name == "return_west_lane") {
      sendGoal("dock_approach", -2.6, 0.8, 0.0);
    }

    else if (goal_name == "dock_approach") {
      sendGoal("charging_dock", 0.0, 1.0, 3.14);
    }

    else if (goal_name == "charging_dock") {
      setState(MissionState::MissionComplete);
      RCLCPP_INFO(this->get_logger(), "Mission complete");
      rclcpp::shutdown();
    }
  }

  double completionTolerance(const std::string & goal_name) const
  {
    if (goal_name == "delivery_zone") {
      return strict_goal_tolerance_;
    }

    if (goal_name == "charging_dock") {
      return strict_goal_tolerance_;
    }

    if (
      goal_name == "pickup_south_lane" ||
      goal_name == "delivery_approach" ||
      goal_name == "delivery_lane_mid" ||
      goal_name == "return_south_lane" ||
      goal_name == "return_mid_lane" ||
      goal_name == "return_west_lane" ||
      goal_name == "dock_approach")
    {
      return loose_waypoint_tolerance_;
    }

    return goal_completion_tolerance_;
  }

  double distanceToGoal(
    const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::Pose & goal_pose) const
  {
    const double dx = current_pose.position.x - goal_pose.position.x;
    const double dy = current_pose.position.y - goal_pose.position.y;
    return std::hypot(dx, dy);
  }

  void scheduleNextStep(
    std::chrono::seconds delay,
    std::function<void()> callback)
  {
    pause_timer_ = this->create_wall_timer(
      delay,
      [this, callback]() {
        pause_timer_->cancel();
        callback();
      }
    );
  }

  void setState(MissionState state)
  {
    current_state_ = state;
    RCLCPP_INFO(
      this->get_logger(),
      "Mission state: %s",
      stateName(current_state_).c_str()
    );
  }

  std::string stateName(MissionState state) const
  {
    switch (state) {
      case MissionState::Idle:
        return "IDLE";
      case MissionState::GoingToPickup:
        return "GOING_TO_PICKUP";
      case MissionState::PickingUpPackage:
        return "PICKING_UP_PACKAGE";
      case MissionState::GoingToDropoff:
        return "GOING_TO_DROPOFF";
      case MissionState::DeliveringPackage:
        return "DELIVERING_PACKAGE";
      case MissionState::ReturningToDock:
        return "RETURNING_TO_DOCK";
      case MissionState::MissionComplete:
        return "MISSION_COMPLETE";
      case MissionState::Failed:
        return "FAILED";
    }

    return "UNKNOWN";
  }

  MissionState current_state_ = MissionState::Idle;
  bool active_goal_has_moved_ = false;
  double goal_completion_tolerance_;
  double loose_waypoint_tolerance_;
  double strict_goal_tolerance_;
  geometry_msgs::msg::Pose active_goal_pose_;
  std::unordered_set<std::string> completed_by_feedback_goals_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr pause_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
