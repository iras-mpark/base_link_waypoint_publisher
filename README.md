# Local LiDAR Planner Workspace

This workspace contains a stripped-down ROS 2 package `local_lidar_planner` that performs purely local, LiDAR-based obstacle avoidance. The package was extracted from the main autonomy stack so it can be versioned separately and iterated on without coupling to global-mapping logic.

## Layout

- `src/local_lidar_planner`: ROS 2 package with source, launch files, and the precomputed path library copied from the original stack.

You can build it with:

```bash
cd /root/local_lidar_planner_ws
colcon build --packages-select local_lidar_planner
source install/setup.bash
ros2 launch local_lidar_planner local_lidar_planner.launch
```

Adjust dependencies or launch arguments to match your hardware (e.g., remap `/registered_scan`, inject your custom waypoint source, etc.).

## Lightweight Python Nodes

For quick bring-up or simulation runs where no joystick, injected obstacles, or external stop logic are available, the package now includes minimalist Python nodes:

- `local_lidar_planner_simple.py`: takes `/utlidar/cloud` plus a TF transform from `path_frame` (default `base_link`) to `goal_tf_frame` (e.g., `suitcase_frame`), then builds a path that keeps the robot facing the target but stops `goal_offset` meters short (default 1 m) while rejecting blocked headings.
- `path_follower_simple.py`: consumes the simple path, scales speed with goal distance (up to `max_speed`), and publishes `/cmd_vel` without touching `/joy`, `/speed`, or `/stop`. When `is_real_robot=true`, it also mirrors the command into `/api/sport/request` using the same MOVE/STOP payloads as the C++ follower.

Launch both with:

```bash
ros2 launch local_lidar_planner local_lidar_planner_simple.launch
```

All original C++ nodes remain untouched for full-feature deployments.

> **LiDAR input:** Both the C++ and Python planners now default to reading `/utlidar/cloud` directly and assume the scan is expressed in `base_link`. Override the topic or target frame via the `laserTopic` / `laserFixedFrame` and `scan_topic` parameters if your setup differs.
