# Delivery Robot Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Improve the ROS 2 delivery robot simulation with C++ safety behavior, clearer mission execution, tests, and GitHub-ready documentation.

**Architecture:** Keep the current package layout. Upgrade the perception safety node into the final velocity gate using person detection and LaserScan ranges, improve mission manager state/log handling, and document the verified runtime path.

**Tech Stack:** ROS 2 Jazzy, C++17, Nav2, Gazebo, Python launch/tests, pytest/ament.

---

### Task 1: Safety Gate Tests

**Files:**
- Modify: `src/delivery_bot_bringup/test/test_velocity_topic_chain.py`

- [ ] Add assertions that `safety_stop_node.cpp` subscribes to `sensor_msgs/msg/laser_scan.hpp`, `/scan`, declares stop/slowdown thresholds, and clamps velocity.
- [ ] Run `colcon test --packages-select delivery_bot_bringup --event-handlers console_direct+`.

### Task 2: C++ Safety Controller

**Files:**
- Modify: `src/delivery_bot_perception_cpp/src/safety_stop_node.cpp`

- [ ] Add LaserScan subscription on `/scan`.
- [ ] Add parameters for `stop_distance`, `slow_distance`, `caution_speed_scale`, and `cmd_timeout_ms`.
- [ ] Publish zero velocity when a person is detected or scan obstacle distance is inside stop range.
- [ ] Scale forward velocity in the caution range while preserving angular command.

### Task 3: Mission Manager Logs

**Files:**
- Modify: `src/delivery_bot_mission_cpp/src/mission_manager.cpp`

- [ ] Replace stringly mission state with enum-backed state names.
- [ ] Add non-blocking pause timers for pickup/dropoff waits.
- [ ] Log state transitions and goal details for demo output.

### Task 4: GitHub Documentation

**Files:**
- Create: `README.md`

- [ ] Document packages, architecture, build/test commands, and runtime commands.
- [ ] Include the safe velocity chain and GitHub upload commands.

### Task 5: Verification

- [ ] Run `colcon build`.
- [ ] Run `colcon test`.
- [ ] Summarize working output and any limits.
