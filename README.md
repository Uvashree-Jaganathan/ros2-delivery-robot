# Autonomous Indoor Delivery Robot Simulation

ROS 2 Jazzy workspace for a simulated autonomous mobile robot that performs an indoor delivery mission in Gazebo with Nav2, C++ mission control, and a perception-aware safety velocity gate.

## Packages

- `robot_description`: URDF/Xacro model with differential drive, lidar, camera link, and delivery box.
- `delivery_bot_gazebo`: Gazebo indoor delivery world.
- `delivery_bot_bringup`: simulation launch and ROS-Gazebo topic bridges.
- `delivery_bot_navigation`: Nav2 launch files, map, and navigation parameters.
- `delivery_bot_mission_cpp`: C++ waypoint mission manager for pickup, delivery, and dock return.
- `delivery_bot_perception_cpp`: C++ YOLO ONNX perception node and safety-stop velocity gate.

## Architecture

The robot uses Nav2 for path planning and trajectory generation. Gazebo only receives commands after they pass through the C++ safety gate:

```text
Nav2 velocity smoother -> /cmd_vel_smoothed
safety_stop_node -> /cmd_vel_safe
Gazebo bridge + diff-drive plugin -> /cmd_vel_safe
```

`safety_stop_node` stops the robot when a person is detected or lidar reports an obstacle inside the stop zone. It slows forward motion in the caution zone and publishes zero velocity if Nav2 command input times out.

## Build And Test

Run from the workspace root:

```bash
colcon build
colcon test
```

After building, source the workspace:

```bash
source install/setup.bash
```

## Run The Simulation

Use separate terminals.

Terminal 1:

```bash
source install/setup.bash
ros2 launch delivery_bot_bringup simulation.launch.py
```

Terminal 2:

```bash
source install/setup.bash
ros2 launch delivery_bot_navigation navigation.launch.py
```

Wait for Nav2 managed nodes to become active, then run the mission.

Terminal 3:

```bash
source install/setup.bash
ros2 run delivery_bot_mission_cpp mission_manager
```

Expected mission log sequence:

```text
Mission state: GOING_TO_PICKUP
Sending goal: pickup_zone
Mission state: PICKING_UP_PACKAGE
Mission state: GOING_TO_DROPOFF
Sending goal: delivery_approach
Sending goal: delivery_zone
Mission state: DELIVERING_PACKAGE
Mission state: RETURNING_TO_DOCK
Sending goal: charging_dock
Mission state: MISSION_COMPLETE
Mission complete
```

## GitHub Upload

This workspace can be uploaded after build and tests pass:

```bash
git init
git add README.md AGENTS.md docs src
git commit -m "Add autonomous indoor delivery robot simulation"
git branch -M main
git remote add origin https://github.com/<your-username>/<repo-name>.git
git push -u origin main
```

Do not commit generated `build/`, `install/`, `log/`, `.pytest_cache/`, or large duplicate model artifacts unless you intentionally want them in the repository.
