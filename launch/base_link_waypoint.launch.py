from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_arg(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description() -> LaunchDescription:
    parent_frame = LaunchConfiguration("parent_frame")
    base_link_frame = LaunchConfiguration("base_link_frame")
    target_frame = LaunchConfiguration("target_frame")
    global_frame = LaunchConfiguration("global_frame")
    waypoint_topic = LaunchConfiguration("waypoint_topic")
    publish_period = LaunchConfiguration("publish_period")
    stop_distance = LaunchConfiguration("stop_distance")

    node = Node(
        package="base_link_waypoint_publisher",
        executable="base_link_waypoint_node",
        name="base_link_waypoint_node",
        output="screen",
        parameters=[{
            "parent_frame": parent_frame,
            "base_link_frame": base_link_frame,
            "target_frame": target_frame,
            "global_frame": global_frame,
            "waypoint_topic": waypoint_topic,
            "publish_period": publish_period,
            "stop_distance": stop_distance,
        }],
    )

    return LaunchDescription([
        declare_arg("parent_frame", "vehicle", "Parent frame for the static TF."),
        declare_arg("base_link_frame", "base_link", "Child frame for the static TF."),
        declare_arg("target_frame", "suitcase_frame", "Frame the vehicle approaches."),
        declare_arg("global_frame", "map", "Frame in which the waypoint is published."),
        declare_arg("waypoint_topic", "/way_point", "Topic for waypoint publishing."),
        declare_arg("publish_period", "0.2", "Timer period (seconds) between publishes."),
        declare_arg("stop_distance", "1.0", "Desired stand-off distance (meters)."),
        node,
    ])
