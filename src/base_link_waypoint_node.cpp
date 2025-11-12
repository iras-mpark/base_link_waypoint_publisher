#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif __has_include(<tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class BaseLinkWaypointNode : public rclcpp::Node
{
public:
  BaseLinkWaypointNode()
  : Node("base_link_waypoint_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    static_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
    arrived_(false),
    target_stamp_initialized_(false)
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "vehicle");
    base_frame_ = this->declare_parameter<std::string>("base_link_frame", "base_link");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "suitcase_frame");
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    waypoint_topic_ = this->declare_parameter<std::string>("waypoint_topic", "/way_point");
    publish_period_ = this->declare_parameter<double>("publish_period", 0.2);
    stop_distance_ = this->declare_parameter<double>("stop_distance", 1.0);
    target_timeout_ = this->declare_parameter<double>("target_timeout", 0.5);

    waypoint_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic_, rclcpp::QoS(5));

    declare_sim_time();

    publish_static_identity();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      std::bind(&BaseLinkWaypointNode::timer_cb, this));
  }

private:
  void publish_static_identity()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    static_broadcaster_->sendTransform(tf);
  }

  geometry_msgs::msg::PointStamped make_hold_waypoint(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PointStamped hold;
    hold.header.frame_id = base_frame_;
    hold.header.stamp = stamp;
    hold.point.x = 0.0;
    hold.point.y = 0.0;
    hold.point.z = 0.0;
    return hold;
  }

  void timer_cb()
  {
    const auto now = this->get_clock()->now();
    geometry_msgs::msg::PointStamped waypoint_in_base = make_hold_waypoint(now);
    geometry_msgs::msg::PointStamped waypoint_in_global;
    bool target_available = true;

    try {
      const auto base_to_target = tf_buffer_.lookupTransform(
        base_frame_, target_frame_, tf2::TimePointZero);
      const double dx = base_to_target.transform.translation.x;
      const double dy = base_to_target.transform.translation.y;
      const double dist_xy = std::hypot(dx, dy);
      const auto stamp = rclcpp::Time(base_to_target.header.stamp);
      const bool stamp_valid = stamp.nanoseconds() > 0;
      const double age = stamp_valid ? (now - stamp).seconds() : 0.0;
      bool stamp_progressed = true;

      if (stamp_valid) {
        if (!target_stamp_initialized_ || stamp != last_target_stamp_) {
          last_target_stamp_ = stamp;
          target_stamp_initialized_ = true;
        } else {
          stamp_progressed = false;
        }
      } else {
        target_stamp_initialized_ = false;
      }

      const bool target_stale = target_timeout_ > 0.0 && stamp_valid && age > target_timeout_;
      if (target_stale && stamp_progressed) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Transform %s -> %s is stale (%.2fs > %.2fs) but using it for waypoint generation.",
          base_frame_.c_str(), target_frame_.c_str(), age, target_timeout_);
      }

      const bool target_lost =
        target_timeout_ > 0.0 && stamp_valid && !stamp_progressed && age > target_timeout_;
      if (target_lost) {
        target_available = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Transform %s -> %s stopped updating (%.2fs without change); holding waypoint.",
          base_frame_.c_str(), target_frame_.c_str(), age);
      }

      if (!target_available) {
        arrived_ = false;
      } else if (dist_xy <= stop_distance_) {
        arrived_ = true;
        waypoint_in_base.point.x = 0.0;
        waypoint_in_base.point.y = 0.0;
        waypoint_in_base.point.z = 0.0;
      } else {
        arrived_ = false;
        const double desired = dist_xy - stop_distance_;
        const double scale = desired / dist_xy;
        waypoint_in_base.point.x = dx * scale;
        waypoint_in_base.point.y = dy * scale;
        waypoint_in_base.point.z = 0.0;
      }
    } catch (const tf2::TransformException & ex) {
      arrived_ = false;
      target_available = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to lookup %s -> %s: %s. Publishing hold waypoint on %s.",
        base_frame_.c_str(), target_frame_.c_str(), ex.what(), waypoint_topic_.c_str());
    }

    try {
      waypoint_in_global = tf_buffer_.transform(
        waypoint_in_base, global_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to transform waypoint to %s: %s",
        global_frame_.c_str(), ex.what());
      return;
    }

    waypoint_pub_->publish(waypoint_in_global);

    if (arrived_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Within %.2f m of %s, publishing hold waypoint.",
        stop_distance_, target_frame_.c_str());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool arrived_;
  std::string parent_frame_;
  std::string base_frame_;
  std::string target_frame_;
  std::string global_frame_;
  std::string waypoint_topic_;
  double publish_period_;
  double stop_distance_;
  double target_timeout_;
  rclcpp::Time last_target_stamp_;
  bool target_stamp_initialized_;

  void declare_sim_time()
  {
    try {
      this->declare_parameter<bool>("use_sim_time", false);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      // Parameter already declared elsewhere; respect the existing value.
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseLinkWaypointNode>());
  rclcpp::shutdown();
  return 0;
}
