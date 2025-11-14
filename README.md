# Local LiDAR Planner Workspace

This workspace contains a stripped-down ROS 2 package `local_lidar_planner` that performs purely local, LiDAR-based obstacle avoidance. The package was extracted from the main autonomy stack so it can be versioned separately and iterated on without coupling to global-mapping logic. All legacy C++ nodes have been removed so the workspace only ships the lightweight Python pipeline.

## Layout

- `src/local_lidar_planner`: ROS 2 package with launch files and the Python planner/follower pair.

You can build it with:

```bash
cd /root/local_lidar_planner_ws
colcon build --packages-select local_lidar_planner
source install/setup.bash
ros2 launch local_lidar_planner local_lidar_planner.launch
```

That launch file brings up the **simple** Python planner/follower pair:

- `local_lidar_planner_simple.py`: takes `/utlidar/cloud` plus a TF transform from `path_frame` (default `base_link`) to `goal_tf_frame` (e.g., `suitcase_frame`), then builds a path that keeps the robot facing the target but stops `goal_offset` meters short (default 1 m) while rejecting blocked headings. It also publishes `/goal_preview` in `base_link` for quick RViz visualization.
- `path_follower_simple.py`: consumes the simple path, scales speed with goal distance (up to `max_speed`), and publishes `/cmd_vel` without touching `/joy`, `/speed`, or `/stop`. When `is_real_robot=true`, it mirrors the command into `/api/sport/request` using the same MOVE/STOP payloads as the C++ follower.

> **LiDAR input:** The simple planner defaults to reading `/utlidar/cloud` and assumes the scan is expressed in `base_link`. If your LiDAR publishes in another frame (e.g., `utlidar_lidar`), set the `lidar_*` launch arguments to inject the correct static transform into TF.
