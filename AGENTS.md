# Agent Instructions

## Project Context

This is a ROS 2 Jazzy delivery robot workspace. The main packages are:

- `robot_description`: robot URDF/Xacro and display launch files.
- `delivery_bot_gazebo`: Gazebo world assets.
- `delivery_bot_bringup`: simulation launch and ROS-Gazebo bridges.
- `delivery_bot_navigation`: Nav2 launch, maps, and parameters.
- `delivery_bot_mission_cpp`: mission manager that sends Nav2 goals.
- `delivery_bot_perception_cpp`: perception and safety-stop nodes.

## Build And Test

Run commands from `/home/uva/delivery_robot`.

```bash
colcon build
colcon test
```

After building, source the workspace before running ROS commands:

```bash
source install/setup.bash
```

## Runtime Commands

Use separate terminals:

```bash
ros2 launch delivery_bot_bringup simulation.launch.py
ros2 launch delivery_bot_navigation navigation.launch.py
ros2 run delivery_bot_mission_cpp mission_manager
```

Wait for Nav2 to report `Managed nodes are active` before starting the mission manager.

## Navigation Notes

- Gazebo currently consumes Nav2's smoothed velocity output on `/cmd_vel_smoothed`.
- The simulation bridge must include `/cmd_vel_smoothed@geometry_msgs/msg/Twist]gz.msgs.Twist`.
- The diff-drive plugin topic in `robot_description/urdf/delivery_bot.urdf.xacro` should match the bridge topic.
- Mission waypoints are defined in `delivery_bot_mission_cpp/src/mission_manager.cpp`.
- Nav2 parameters are in `delivery_bot_navigation/config/nav2_params.yaml`.

## Editing Guidelines

- Keep changes scoped to the relevant ROS package.
- Prefer existing package structure and launch patterns.
- Rebuild after changing URDF, launch files, Nav2 parameters, or C++ nodes.
- Run `colcon test` before reporting that a change is complete.
- Do not edit generated `build/`, `install/`, or `log/` outputs directly.
