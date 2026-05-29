# Copilot Instructions

## Project Context

This repository is a ROS 2 Jazzy delivery robot workspace. Work from the repository root at `/home/uva/delivery_robot`.

Main packages:

- `robot_description`: robot URDF/Xacro and display launch files.
- `delivery_bot_gazebo`: Gazebo world assets.
- `delivery_bot_bringup`: simulation launch and ROS-Gazebo bridge setup.
- `delivery_bot_navigation`: Nav2 launch files, maps, and parameters.
- `delivery_bot_mission_cpp`: C++ mission manager that sends Nav2 goals.
- `delivery_bot_perception_cpp`: C++ perception and safety-stop nodes.

## Build And Test

Use these commands from the workspace root:

```bash
colcon build
colcon test
```

After building, source the workspace before running ROS commands:

```bash
source install/setup.bash
```

Rebuild after changing URDF, launch files, Nav2 parameters, CMake/package metadata, or C++ nodes.

## Runtime

Use separate terminals:

```bash
ros2 launch delivery_bot_bringup simulation.launch.py
ros2 launch delivery_bot_navigation navigation.launch.py
ros2 run delivery_bot_mission_cpp mission_manager
```

Wait for Nav2 to report `Managed nodes are active` before starting the mission manager.

## Navigation And Simulation Notes

- Gazebo consumes Nav2's smoothed velocity output on `/cmd_vel_smoothed`.
- The ROS-Gazebo bridge must include `/cmd_vel_smoothed@geometry_msgs/msg/Twist]gz.msgs.Twist`.
- The diff-drive plugin topic in `robot_description/urdf/delivery_bot.urdf.xacro` should match the bridged command velocity topic.
- Mission waypoints are defined in `delivery_bot_mission_cpp/src/mission_manager.cpp`.
- Nav2 parameters are in `delivery_bot_navigation/config/nav2_params.yaml`.

## Editing Guidelines

- Keep changes scoped to the relevant ROS package.
- Prefer the existing package structure, launch patterns, and CMake style.
- Do not edit generated `build/`, `install/`, or `log/` outputs directly.
- Do not commit generated Python cache files.
- When reporting completion, mention whether `colcon build` and `colcon test` were run.
