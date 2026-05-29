from pathlib import Path


def test_safety_stop_is_final_velocity_gate():
    workspace_src = Path(__file__).resolve().parents[2]

    safety_stop = (
        workspace_src
        / 'delivery_bot_perception_cpp'
        / 'src'
        / 'safety_stop_node.cpp'
    ).read_text()
    robot_urdf = (
        workspace_src
        / 'robot_description'
        / 'urdf'
        / 'delivery_bot.urdf.xacro'
    ).read_text()
    simulation_launch = (
        workspace_src
        / 'delivery_bot_bringup'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text()

    assert '"/cmd_vel_smoothed"' in safety_stop
    assert '"/cmd_vel_safe"' in safety_stop
    assert '<topic>/cmd_vel_safe</topic>' in robot_urdf
    assert '/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist' in simulation_launch
    assert "package='delivery_bot_perception_cpp'" in simulation_launch
    assert "executable='safety_stop_node'" in simulation_launch


def test_safety_stop_publishes_zero_when_nav2_commands_timeout():
    workspace_src = Path(__file__).resolve().parents[2]
    safety_stop = (
        workspace_src
        / 'delivery_bot_perception_cpp'
        / 'src'
        / 'safety_stop_node.cpp'
    ).read_text()

    assert 'cmd_timeout_' in safety_stop
    assert 'create_wall_timer' in safety_stop
    assert 'publishStop()' in safety_stop
    assert 'last_cmd_time_' in safety_stop


def test_safety_stop_uses_laser_scan_to_slow_or_stop_for_obstacles():
    workspace_src = Path(__file__).resolve().parents[2]
    safety_stop = (
        workspace_src
        / 'delivery_bot_perception_cpp'
        / 'src'
        / 'safety_stop_node.cpp'
    ).read_text()

    assert 'sensor_msgs/msg/laser_scan.hpp' in safety_stop
    assert '"/scan"' in safety_stop
    assert 'stop_distance' in safety_stop
    assert 'slow_distance' in safety_stop
    assert 'caution_speed_scale' in safety_stop
    assert 'nearest_obstacle_distance_' in safety_stop


def test_safety_stop_only_uses_forward_scan_sector_for_distance_gate():
    workspace_src = Path(__file__).resolve().parents[2]
    safety_stop = (
        workspace_src
        / 'delivery_bot_perception_cpp'
        / 'src'
        / 'safety_stop_node.cpp'
    ).read_text()
    simulation_launch = (
        workspace_src
        / 'delivery_bot_bringup'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text()

    assert 'scan_forward_angle' in safety_stop
    assert 'angle_min + static_cast<double>(i) * angle_increment' in safety_stop
    assert 'std::abs(angle) > scan_forward_angle_' in safety_stop
    assert "'scan_forward_angle': 0.90" in simulation_launch


def test_safety_stop_preserves_rotation_for_nav2_obstacle_avoidance():
    workspace_src = Path(__file__).resolve().parents[2]
    safety_stop = (
        workspace_src
        / 'delivery_bot_perception_cpp'
        / 'src'
        / 'safety_stop_node.cpp'
    ).read_text()
    simulation_launch = (
        workspace_src
        / 'delivery_bot_bringup'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text()

    assert 'makeAvoidanceCommand' in safety_stop
    assert 'output.angular.z = input.angular.z' in safety_stop
    assert 'person_detected_' in safety_stop
    assert "'stop_distance': 0.55" in simulation_launch
    assert "'slow_distance': 1.00" in simulation_launch
    assert "'caution_speed_scale': 0.45" in simulation_launch


def test_safety_stop_injects_recovery_turn_when_nav2_is_not_turning():
    workspace_src = Path(__file__).resolve().parents[2]
    safety_stop = (
        workspace_src
        / 'delivery_bot_perception_cpp'
        / 'src'
        / 'safety_stop_node.cpp'
    ).read_text()
    simulation_launch = (
        workspace_src
        / 'delivery_bot_bringup'
        / 'launch'
        / 'simulation.launch.py'
    ).read_text()

    assert 'recovery_turn_angular_speed' in safety_stop
    assert 'min_turn_command' in safety_stop
    assert 'nearest_left_obstacle_distance_' in safety_stop
    assert 'nearest_right_obstacle_distance_' in safety_stop
    assert 'std::abs(output.angular.z) < min_turn_command_' in safety_stop
    assert "'recovery_turn_angular_speed': 0.45" in simulation_launch


def test_mission_manager_accepts_close_waypoint_before_nav2_overshoots():
    workspace_src = Path(__file__).resolve().parents[2]
    mission_manager = (
        workspace_src
        / 'delivery_bot_mission_cpp'
        / 'src'
        / 'mission_manager.cpp'
    ).read_text()

    assert 'goal_completion_tolerance' in mission_manager
    assert 'completed_by_feedback_goals_' in mission_manager
    assert 'return loose_waypoint_tolerance_' in mission_manager
    assert 'distanceToGoal(feedback->current_pose.pose, active_goal_pose_)' in mission_manager
    assert 'goal_distance <= completionTolerance(goal_name)' in mission_manager
    assert 'nav_client_->async_cancel_goal(goal_handle)' in mission_manager


def test_mission_manager_requires_tight_delivery_zone_before_package_delivered():
    workspace_src = Path(__file__).resolve().parents[2]
    mission_manager = (
        workspace_src
        / 'delivery_bot_mission_cpp'
        / 'src'
        / 'mission_manager.cpp'
    ).read_text()

    assert 'strict_goal_tolerance_' in mission_manager
    assert 'if (goal_name == "delivery_zone")' in mission_manager
    assert 'return strict_goal_tolerance_' in mission_manager
    assert 'goal_completion_tolerance",\n      0.30' in mission_manager
    assert 'RCLCPP_INFO(this->get_logger(), "Package delivered")' in mission_manager


def test_mission_manager_returns_to_dock_through_intermediate_waypoint():
    workspace_src = Path(__file__).resolve().parents[2]
    mission_manager = (
        workspace_src
        / 'delivery_bot_mission_cpp'
        / 'src'
        / 'mission_manager.cpp'
    ).read_text()

    assert 'pickup_south_lane' in mission_manager
    assert 'delivery_lane_mid' in mission_manager
    assert 'return_south_lane' in mission_manager
    assert 'return_mid_lane' in mission_manager
    assert 'return_west_lane' in mission_manager
    assert 'dock_approach' in mission_manager
    assert 'sendGoal("pickup_zone", -2.5, 1.0, 0.0)' in mission_manager
    assert 'sendGoal("pickup_south_lane", -2.5, -3.4, 0.0)' in mission_manager
    assert 'sendGoal("delivery_lane_mid", 1.6, -3.6, 0.0)' in mission_manager
    assert 'sendGoal("return_south_lane", 3.2, -3.8, 3.14)' in mission_manager
    assert 'sendGoal("return_mid_lane", 0.0, -3.8, 3.14)' in mission_manager
    assert 'sendGoal("return_west_lane", -2.6, -3.0, 1.57)' in mission_manager
    assert 'sendGoal("dock_approach", -2.6, 0.8, 0.0)' in mission_manager
    assert 'sendGoal("charging_dock", 0.0, 1.0, 3.14)' in mission_manager
