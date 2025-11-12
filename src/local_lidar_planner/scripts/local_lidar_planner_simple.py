#!/usr/bin/env python3
"""Lightweight LiDAR-only local planner.

This node keeps the original C++ planner available but offers a drastically
simplified fallback that:
  * drives straight toward the waypoint whenever possible,
  * rejects headings that contain LiDAR returns inside a configurable safety
    radius, and
  * publishes a short local path in the vehicle frame.

It intentionally ignores joystick commands, external obstacle injections, and
other side channels so it can run standalone in minimal deployments.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

class SimpleLocalPlanner(Node):
    """Generate a short collision-free path directly toward the waypoint."""

    def __init__(self) -> None:
        super().__init__("local_lidar_planner_simple")

        # Path construction parameters
        self.declare_parameter("path_frame", "vehicle")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("path_resolution", 0.25)
        self.declare_parameter("max_path_length", 3.0)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("goal_offset", 1.0)

        # Obstacle handling parameters
        self.declare_parameter("num_heading_bins", 72)
        self.declare_parameter("safety_radius", 0.45)
        self.declare_parameter("heading_clearance_deg", 8.0)
        self.declare_parameter("max_considered_range", 4.0)
        self.declare_parameter("obstacle_z_min", -1.0)
        self.declare_parameter("obstacle_z_max", 1.0)
        self.declare_parameter("scan_topic", "/utlidar/cloud")
        self.declare_parameter("goal_tf_frame", "")
        self.declare_parameter("goal_tf_timeout", 0.5)
        self.declare_parameter("obstacle_topic", "/local_obstacles")

        self.path_frame: str = self.get_parameter("path_frame").get_parameter_value().string_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.path_resolution = self.get_parameter("path_resolution").get_parameter_value().double_value
        self.max_path_length = self.get_parameter("max_path_length").get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter("goal_tolerance").get_parameter_value().double_value
        self.goal_offset = self.get_parameter("goal_offset").get_parameter_value().double_value
        self.num_heading_bins = int(self.get_parameter("num_heading_bins").get_parameter_value().integer_value)
        self.safety_radius = self.get_parameter("safety_radius").get_parameter_value().double_value

        heading_clearance = math.radians(
            self.get_parameter("heading_clearance_deg").get_parameter_value().double_value
        )
        self.heading_clearance_bins = max(1, int(round(heading_clearance / (2.0 * math.pi / self.num_heading_bins))))

        self.max_considered_range = self.get_parameter("max_considered_range").get_parameter_value().double_value
        self.obstacle_z_min = self.get_parameter("obstacle_z_min").get_parameter_value().double_value
        self.obstacle_z_max = self.get_parameter("obstacle_z_max").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.goal_tf_frame = self.get_parameter("goal_tf_frame").get_parameter_value().string_value
        if not self.goal_tf_frame:
            raise ValueError("goal_tf_frame parameter must be set (e.g., 'suitcase_frame').")
        self.goal_tf_timeout = self.get_parameter("goal_tf_timeout").get_parameter_value().double_value

        self.bin_ranges: List[float] = [math.inf for _ in range(self.num_heading_bins)]
        self.last_scan_time = self.get_clock().now()

        # TF tracking
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, qos=5)
        self._last_tf_warn_time = 0.0

        # Interfaces
        self.create_subscription(PointCloud2, self.scan_topic, self._scan_callback, 5)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_preview", 5)
        self.goal_raw_pub = self.create_publisher(PointStamped, "/goal_raw", 5)
        obstacle_topic = self.get_parameter("obstacle_topic").get_parameter_value().string_value
        self.obstacle_pub = self.create_publisher(PointCloud2, obstacle_topic, 5)

        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1e-3), self._on_timer)
        self.get_logger().info("Simple local planner ready (LiDAR-only, joystick-free).")

    # ------------------------------------------------------------------ Callbacks
    def _scan_callback(self, cloud: PointCloud2) -> None:
        """Convert the incoming point cloud into a coarse polar histogram."""
        self.bin_ranges = [math.inf for _ in range(self.num_heading_bins)]
        had_points = False
        obstacle_points: List[Tuple[float, float, float]] = []

        for point in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if not (self.obstacle_z_min <= z <= self.obstacle_z_max):
                continue

            distance = math.hypot(x, y)
            if distance < 1e-3 or distance > self.max_considered_range:
                continue
            heading = math.atan2(y, x)
            idx = self._heading_to_index(heading)
            if distance < self.bin_ranges[idx]:
                self.bin_ranges[idx] = distance
            had_points = True
            obstacle_points.append((x, y, z))

        if had_points:
            self.last_scan_time = self.get_clock().now()
        obstacle_header = cloud.header
        obstacle_header.frame_id = self.path_frame
        if obstacle_points:
            obstacle_msg = point_cloud2.create_cloud_xyz32(obstacle_header, obstacle_points)
        else:
            obstacle_msg = PointCloud2()
            obstacle_msg.header = obstacle_header
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_pub.publish(obstacle_msg)

    # ------------------------------------------------------------------ Timer
    def _on_timer(self) -> None:
        path_msg = self._build_path()
        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------ Helpers
    def _build_path(self) -> Path:
        """Return a short path that respects the latest goal and LiDAR data."""
        now = self.get_clock().now()
        path = Path()
        path.header.stamp = now.to_msg()
        path.header.frame_id = self.path_frame

        rel_goal = self._lookup_goal_in_vehicle(now)
        if rel_goal is None:
            self._publish_goal_marker(0.0, 0.0, now)
            path.poses.append(self._pose_at(0.0, 0.0, 0.0, now))
            return path

        raw_distance = math.hypot(rel_goal[0], rel_goal[1])
        desired_heading = math.atan2(rel_goal[1], rel_goal[0])
        self._publish_goal_marker(rel_goal[0], rel_goal[1], now)

        if raw_distance <= self.goal_offset:
            path.poses.append(self._pose_at(0.0, 0.0, desired_heading, now))
            return path

        goal_distance = max(0.0, raw_distance - self.goal_offset)

        if goal_distance < self.goal_tolerance:
            path.poses.append(self._pose_at(0.0, 0.0, desired_heading, now))
            return path
        heading = self._select_heading(desired_heading)

        if heading is None:
            # No clear heading: stop in place
            path.poses.append(self._pose_at(0.0, 0.0, desired_heading, now))
            return path

        path_length = min(goal_distance, self.max_path_length)
        num_segments = max(1, int(math.ceil(path_length / self.path_resolution)))

        for i in range(1, num_segments + 1):
            distance = min(i * self.path_resolution, path_length)
            x = math.cos(heading) * distance
            y = math.sin(heading) * distance
            path.poses.append(self._pose_at(x, y, heading, now))

        return path

    def _lookup_goal_in_vehicle(self, stamp: Time) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.goal_tf_frame,
                Time(),
                timeout=Duration(seconds=self.goal_tf_timeout),
            )
            rel_x = transform.transform.translation.x
            rel_y = transform.transform.translation.y
            raw_msg = PointStamped()
            raw_msg.header.stamp = stamp.to_msg()
            raw_msg.header.frame_id = self.path_frame
            raw_msg.point.x = rel_x
            raw_msg.point.y = rel_y
            self.goal_raw_pub.publish(raw_msg)
            return rel_x, rel_y
        except TransformException as exc:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_tf_warn_time > 2.0:
                self.get_logger().warn(f"TF lookup failed ({self.goal_tf_frame}): {exc}")
                self._last_tf_warn_time = now_sec
            return None

    def _select_heading(self, desired: float) -> Optional[float]:
        if self._is_heading_clear(desired):
            return desired

        bin_step = 1
        while bin_step <= self.num_heading_bins // 2:
            offset = bin_step * (2.0 * math.pi / self.num_heading_bins)
            for direction in (1, -1):
                candidate = self._normalize_angle(desired + direction * offset)
                if self._is_heading_clear(candidate):
                    return candidate
            bin_step += 1
        return None

    def _is_heading_clear(self, heading: float) -> bool:
        idx = self._heading_to_index(heading)
        bins_to_check = range(
            max(0, idx - self.heading_clearance_bins), min(self.num_heading_bins, idx + self.heading_clearance_bins + 1)
        )
        recent_scan = (self.get_clock().now() - self.last_scan_time) < Duration(seconds=1.0)
        if not recent_scan:
            return True

        for i in bins_to_check:
            if self.bin_ranges[i] < self.safety_radius:
                return False
        return True

    def _heading_to_index(self, heading: float) -> int:
        normalized = self._normalize_angle(heading)
        fraction = (normalized + math.pi) / (2.0 * math.pi)
        idx = int(fraction * self.num_heading_bins) % self.num_heading_bins
        return idx

    def _pose_at(self, x: float, y: float, heading: float, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.path_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)
        return pose

    def _publish_goal_marker(self, x: float, y: float, stamp) -> None:
        goal = PointStamped()
        goal.header.stamp = stamp.to_msg()
        goal.header.frame_id = self.path_frame
        goal.point.x = x
        goal.point.y = y
        self.goal_pub.publish(goal)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimpleLocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
