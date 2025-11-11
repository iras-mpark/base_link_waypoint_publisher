#!/usr/bin/env python3
"""Publish the LiDAR-to-base transform using a StaticTransformBroadcaster."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class UTLidarTFBroadcaster(Node):
    """Broadcast a static LiDAR -> path_frame transform."""

    def __init__(self) -> None:
        super().__init__("utlidar_tf_broadcaster")
        self.declare_parameter("lidar_frame", "utlidar_lidar")
        self.declare_parameter("path_frame", "base_link")
        self.declare_parameter("lidar_x", 0.0)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", 0.0)
        self.declare_parameter("lidar_roll", 0.0)
        self.declare_parameter("lidar_pitch", 0.0)
        self.declare_parameter("lidar_yaw", 0.0)

        lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value
        path_frame = self.get_parameter("path_frame").get_parameter_value().string_value
        tx = self.get_parameter("lidar_x").get_parameter_value().double_value
        ty = self.get_parameter("lidar_y").get_parameter_value().double_value
        tz = self.get_parameter("lidar_z").get_parameter_value().double_value
        roll = self.get_parameter("lidar_roll").get_parameter_value().double_value
        pitch = self.get_parameter("lidar_pitch").get_parameter_value().double_value
        yaw = self.get_parameter("lidar_yaw").get_parameter_value().double_value

        quat = quaternion_from_euler(roll, pitch, yaw)

        transform = TransformStamped()
        transform.header.frame_id = lidar_frame
        transform.child_frame_id = path_frame
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.translation.z = tz
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Publishing static transform {lidar_frame} -> {path_frame} "
            f"with translation=({tx:.3f},{ty:.3f},{tz:.3f}) "
            f"and RPY=({roll:.3f},{pitch:.3f},{yaw:.3f})"
        )


def main() -> None:
    rclpy.init()
    try:
        node = UTLidarTFBroadcaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
