## base_link_waypoint_publisher

A minimal ROS 2 package that computes a hold waypoint in `map` (or any global frame) so the base link stops a configurable distance before a target TF frame. It also ships the `system_local_lidar_planner.sh` helper that launches the node after sourcing the workspace.

### Build

```bash
cd /path/to/your_ws
colcon build
source install/setup.bash
```

No extra `colcon` options are required.

### Parameters

All parameters are declared on the node and may be overridden via the launch file:

| Name | Default | Description |
| --- | --- | --- |
| `parent_frame` | `vehicle` | Frame used as the parent for the static identity transform. |
| `base_link_frame` | `base_link` | Base link to move toward the target. |
| `target_frame` | `suitcase_frame` | Frame representing the object the robot approaches. |
| `global_frame` | `map` | Frame where waypoints are published. |
| `waypoint_topic` | `/way_point` | Output topic for `geometry_msgs/PointStamped`. |
| `joy_topic` | `/joy` | Topic for single-shot `sensor_msgs/Joy` triggers to enter waypoint mode. |
| `publish_period` | `0.2` | Timer period in seconds. |
| `stop_distance` | `1.0` | Minimum distance (m) to keep from the target. |
| `target_timeout` | `0.5` | Warn if the `target_frame` TF is older than this many seconds (`-1` disables). |
| `use_sim_time` | `false` | Toggle ROS time usage (set to `true` in simulation only). |

### Launching

Use either the ROS 2 launch file or the helper script:

```bash
# Launch directly
ros2 launch base_link_waypoint_publisher base_link_waypoint.launch.py

# Or run the wrapper script (also sources install/setup.bash)
./install/base_link_waypoint_publisher/lib/system_local_lidar_planner.sh
```

`system_local_lidar_planner.sh` tries to start `~/autonomy_stack_go2/system_real_robot.sh` if it exists and then launches `ros2 launch base_link_waypoint_publisher base_link_waypoint.launch.py`.
