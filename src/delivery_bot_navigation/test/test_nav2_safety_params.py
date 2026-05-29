from pathlib import Path

import yaml


def test_manual_goals_require_close_position_and_heading():
    params_path = Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'
    params = yaml.safe_load(params_path.read_text())

    goal_checker = (
        params['controller_server']['ros__parameters']['general_goal_checker']
    )

    assert goal_checker['xy_goal_tolerance'] <= 0.3
    assert goal_checker['yaw_goal_tolerance'] <= 0.5


def test_robot_does_not_drive_backward_for_manual_nav_goals():
    params_path = Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'
    params = yaml.safe_load(params_path.read_text())

    velocity_smoother = params['velocity_smoother']['ros__parameters']

    assert velocity_smoother['min_velocity'][0] == 0.0


def test_controller_rotates_to_new_path_heading_and_checks_collisions():
    params_path = Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'
    params = yaml.safe_load(params_path.read_text())

    controller = (
        params['controller_server']['ros__parameters']['FollowPath']
    )

    assert controller['use_rotate_to_heading'] is True
    assert controller['use_collision_detection'] is True
    assert controller['max_allowed_time_to_collision_up_to_carrot'] >= 1.0
