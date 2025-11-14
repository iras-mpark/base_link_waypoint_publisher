#!/usr/bin/env python3
"""Aggregate LiDAR scans over a short window to densify local observations."""

from __future__ import annotations

from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class SlidingWindowLidarAccumulator(Node):
    """Collect LiDAR scans over a configurable time window and republish them."""

    def __init__(self) -> None:
        super().__init__("lidar_accumulator")
        self.declare_parameter("input_topic", "/utlidar/transformed_cloud")
        self.declare_parameter("output_topic", "/utlidar/accumulated_cloud")
        self.declare_parameter("history_duration", 0.8)
        self.declare_parameter("max_clouds", 10)
        self.declare_parameter("publish_rate_hz", 15.0)

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        history_seconds = self.get_parameter("history_duration").get_parameter_value().double_value
        # Keep the duration positive but allow small windows.
        history_seconds = max(0.01, history_seconds)
        self.history_duration = Duration(seconds=history_seconds)
        self.max_clouds = max(1, self.get_parameter("max_clouds").get_parameter_value().integer_value)
        publish_rate = max(1e-2, self.get_parameter("publish_rate_hz").get_parameter_value().double_value)

        self._history: Deque[Tuple[Time, List[Tuple[float, ...]]]] = deque()
        self._last_frame_id: Optional[str] = None
        self._fields: Optional[List] = None

        self._subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, 10
        )
        self._publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            "LiDAR accumulator ready "
            f"(window={history_seconds:.2f}s, max_clouds={self.max_clouds}, "
            f"{self.input_topic} -> {self.output_topic})"
        )

    # ------------------------------------------------------------------ Helpers
    def _cloud_callback(self, cloud: PointCloud2) -> None:
        """Cache the incoming cloud and immediately prune old entries."""
        raw_points = list(point_cloud2.read_points(cloud, skip_nans=True))
        now = self.get_clock().now()
        if not raw_points:
            self._prune_history(now)
            return

        self._last_frame_id = cloud.header.frame_id or self._last_frame_id
        if self._fields is None:
            # Preserve the original point layout (x/y/z/intensity/...) so downstream nodes stay compatible.
            self._fields = list(cloud.fields)

        cloud_stamp = Time.from_msg(cloud.header.stamp)
        self._history.append((cloud_stamp, raw_points))
        while len(self._history) > self.max_clouds:
            self._history.popleft()
        self._prune_history(now)

    def _prune_history(self, reference_time: Time) -> None:
        """Drop cached scans that fall outside the configured window."""
        while self._history:
            stamp, _ = self._history[0]
            if reference_time - stamp <= self.history_duration:
                break
            self._history.popleft()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        self._prune_history(now)

        accumulated: List[Tuple[float, ...]] = []
        for _, points in self._history:
            accumulated.extend(points)

        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self._last_frame_id or ""

        if accumulated:
            if self._fields:
                msg = point_cloud2.create_cloud(header, self._fields, accumulated)
            else:
                msg = point_cloud2.create_cloud_xyz32(header, accumulated)
        else:
            msg = PointCloud2()
            msg.header = header

        self._publisher.publish(msg)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SlidingWindowLidarAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
