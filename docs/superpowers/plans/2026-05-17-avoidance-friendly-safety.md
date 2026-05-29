# Avoidance Friendly Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let Nav2 avoid obstacles and continue to goals by preventing the safety gate from freezing rotation.

**Architecture:** Keep `safety_stop_node` as the final `/cmd_vel_safe` gate. For close forward obstacles, block forward linear motion but preserve angular velocity so Nav2 can rotate and recover; for caution obstacles, scale linear velocity only. Tune launch thresholds to give Nav2 more room to steer.

**Tech Stack:** ROS 2 Jazzy, C++17, Nav2, Gazebo, pytest/ament.

---

### Task 1: Regression Test

**Files:**
- Modify: `src/delivery_bot_bringup/test/test_velocity_topic_chain.py`

- [ ] Add assertions that `safety_stop_node.cpp` preserves angular velocity while blocking forward motion, and that `simulation.launch.py` uses `stop_distance=0.30`, `slow_distance=0.70`, `caution_speed_scale=0.45`.
- [ ] Run `colcon test --packages-select delivery_bot_bringup --event-handlers console_direct+` and verify the new test fails.

### Task 2: Safety Gate Behavior

**Files:**
- Modify: `src/delivery_bot_perception_cpp/src/safety_stop_node.cpp`
- Modify: `src/delivery_bot_bringup/launch/simulation.launch.py`

- [ ] Add `makeAvoidanceCommand()` that sets linear x/y to zero but preserves angular z from the Nav2 command.
- [ ] Use `makeAvoidanceCommand(*msg)` for close lidar obstacles.
- [ ] Keep full stop for person detection.
- [ ] Scale only linear x/y in the caution zone and preserve angular z.
- [ ] Update launch thresholds.

### Task 3: Verification

- [ ] Run `colcon build --packages-select delivery_bot_perception_cpp delivery_bot_bringup`.
- [ ] Run `colcon test`.
- [ ] Run `colcon test-result --verbose`.
