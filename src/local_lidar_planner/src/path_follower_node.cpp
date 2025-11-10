#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

namespace
{
constexpr double PI = 3.141592653589793;

double sensorOffsetX = 0.0;
double sensorOffsetY = 0.0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double omniDirDiffThre = 1.5;
double noRotSpeed = 10.0;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool manualMode = false;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double goalCloseDis = 1.0;
bool isRealRobot = false;

float joySpeed = 0.0f;
float joySpeedRaw = 0.0f;
float joyYaw = 0.0f;
float joyManualFwd = 0.0f;
float joyManualLeft = 0.0f;
float joyManualYaw = 0.0f;
int safetyStop = 0;

float vehicleX = 0.0f;
float vehicleY = 0.0f;
float vehicleZ = 0.0f;
float vehicleRoll = 0.0f;
float vehiclePitch = 0.0f;
float vehicleYaw = 0.0f;

float vehicleXRec = 0.0f;
float vehicleYRec = 0.0f;
float vehicleZRec = 0.0f;
float vehicleRollRec = 0.0f;
float vehiclePitchRec = 0.0f;
float vehicleYawRec = 0.0f;

float vehicleYawRate = 0.0f;
float vehicleSpeed = 0.0f;

double odomTime = 0.0;
double joyTime = 0.0;
double slowInitTime = 0.0;
double stopInitTime = 0.0;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0.0;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

unitree_api::msg::Request req;
SportClient sportReq;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  const auto &geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = static_cast<float>(roll);
  vehiclePitch = static_cast<float>(pitch);
  vehicleYaw = static_cast<float>(yaw);
  vehicleX = static_cast<float>(odomIn->pose.pose.position.x - std::cos(yaw) * sensorOffsetX + std::sin(yaw) * sensorOffsetY);
  vehicleY = static_cast<float>(odomIn->pose.pose.position.y - std::sin(yaw) * sensorOffsetX - std::cos(yaw) * sensorOffsetY);
  vehicleZ = static_cast<float>(odomIn->pose.pose.position.z);

  if ((std::fabs(roll) > inclThre * PI / 180.0 || std::fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((std::fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
       std::fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) &&
      useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  const int pathSize = static_cast<int>(pathIn->poses.size());
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds();
  joySpeedRaw = std::sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0f) joySpeed = 1.0f;
  if (joy->axes[4] == 0.0f) joySpeed = 0.0f;
  joyYaw = joy->axes[3];
  if (joySpeed == 0.0f && noRotAtStop) joyYaw = 0.0f;

  if (joy->axes[4] < 0.0f && !twoWayDrive) {
    joySpeed = 0.0f;
    joyYaw = 0.0f;
  }

  joyManualFwd = joy->axes[4];
  joyManualLeft = joy->axes[3];
  joyManualYaw = joy->axes[0];

  autonomyMode = joy->axes[2] <= -0.1f;
  manualMode = joy->axes[5] <= -0.1f;
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  const double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0.0f) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0.0f) joySpeed = 0.0f;
    else if (joySpeed > 1.0f) joySpeed = 1.0f;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("omniDirDiffThre", omniDirDiffThre);
  nh->declare_parameter<double>("noRotSpeed", noRotSpeed);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("goalCloseDis", goalCloseDis);
  nh->declare_parameter<bool>("is_real_robot", isRealRobot);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("omniDirDiffThre", omniDirDiffThre);
  nh->get_parameter("noRotSpeed", noRotSpeed);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("goalCloseDis", goalCloseDis);
  nh->get_parameter("is_real_robot", isRealRobot);

  [[maybe_unused]] auto subOdom =
      nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);
  [[maybe_unused]] auto subPath =
      nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);
  [[maybe_unused]] auto subJoystick =
      nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);
  [[maybe_unused]] auto subSpeed =
      nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);
  [[maybe_unused]] auto subStop =
      nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
  auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

  geometry_msgs::msg::TwistStamped cmdVel;
  cmdVel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;
    if (joySpeed < 0.0f) joySpeed = 0.0f;
    else if (joySpeed > 1.0f) joySpeed = 1.0f;
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (pathInit) {
      const float vehicleXRel = std::cos(vehicleYawRec) * (vehicleX - vehicleXRec) +
                                std::sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      const float vehicleYRel = -std::sin(vehicleYawRec) * (vehicleX - vehicleXRec) +
                                 std::cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      const int pathSize = static_cast<int>(path.poses.size());
      const float endDisX = static_cast<float>(path.poses[pathSize - 1].pose.position.x) - vehicleXRel;
      const float endDisY = static_cast<float>(path.poses[pathSize - 1].pose.position.y) - vehicleYRel;
      const float endDis = std::sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX = 0.0f;
      float disY = 0.0f;
      float dis = 0.0f;
      while (pathPointID < pathSize - 1) {
        disX = static_cast<float>(path.poses[pathPointID].pose.position.x) - vehicleXRel;
        disY = static_cast<float>(path.poses[pathPointID].pose.position.y) - vehicleYRel;
        dis = std::sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = static_cast<float>(path.poses[pathPointID].pose.position.x) - vehicleXRel;
      disY = static_cast<float>(path.poses[pathPointID].pose.position.y) - vehicleYRel;
      dis = std::sqrt(disX * disX + disY * disY);
      float pathDir = std::atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        const double timeNow = nh->now().seconds();
        if (std::fabs(dirDiff) > PI / 2 && navFwd && timeNow - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = timeNow;
        } else if (std::fabs(dirDiff) < PI / 2 && !navFwd && timeNow - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = timeNow;
        }
      }

      float joySpeed2 = static_cast<float>(maxSpeed * joySpeed);
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1.0f;
      }

      if (std::fabs(vehicleSpeed) < 2.0f * static_cast<float>(maxAccel / 100.0)) {
        vehicleYawRate = static_cast<float>(-stopYawRateGain * dirDiff);
      } else {
        vehicleYawRate = static_cast<float>(-yawRateGain * dirDiff);
      }

      const double maxYawRateRad = maxYawRate * PI / 180.0;
      if (vehicleYawRate > maxYawRateRad) vehicleYawRate = static_cast<float>(maxYawRateRad);
      else if (vehicleYawRate < -maxYawRateRad) vehicleYawRate = static_cast<float>(-maxYawRateRad);

      if (joySpeed2 == 0.0f && !autonomyMode) {
        vehicleYawRate = static_cast<float>(maxYawRate * joyYaw * PI / 180.0);
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0.0f;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0.0f;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= static_cast<float>(endDis / slowDwnDisThre);
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0.0) {
        joySpeed3 *= static_cast<float>(slowRate1);
      } else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0.0) {
        joySpeed3 *= static_cast<float>(slowRate2);
      }

      if ((std::fabs(dirDiff) < dirDiffThre || (dis < goalCloseDis && std::fabs(dirDiff) < omniDirDiffThre)) &&
          dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += static_cast<float>(maxAccel / 100.0);
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= static_cast<float>(maxAccel / 100.0);
      } else {
        if (vehicleSpeed > 0.0f) vehicleSpeed -= static_cast<float>(maxAccel / 100.0);
        else if (vehicleSpeed < 0.0f) vehicleSpeed += static_cast<float>(maxAccel / 100.0);
      }

      if (std::fabs(vehicleSpeed) > noRotSpeed) vehicleYawRate = 0.0f;

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0.0) {
        vehicleSpeed = 0.0f;
        vehicleYawRate = 0.0f;
      }

      if ((safetyStop & 1) > 0 && vehicleSpeed > 0.0f) vehicleSpeed = 0.0f;
      if ((safetyStop & 2) > 0 && vehicleSpeed < 0.0f) vehicleSpeed = 0.0f;
      if ((safetyStop & 4) > 0 && vehicleYawRate > 0.0f) vehicleYawRate = 0.0f;
      if ((safetyStop & 8) > 0 && vehicleYawRate < 0.0f) vehicleYawRate = 0.0f;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmdVel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        if (std::fabs(vehicleSpeed) <= maxAccel / 100.0) {
          cmdVel.twist.linear.x = 0.0;
          cmdVel.twist.linear.y = 0.0;
        } else {
          cmdVel.twist.linear.x = std::cos(dirDiff) * vehicleSpeed;
          cmdVel.twist.linear.y = -std::sin(dirDiff) * vehicleSpeed;
        }
        cmdVel.twist.angular.z = vehicleYawRate;

        if (manualMode) {
          cmdVel.twist.linear.x = maxSpeed * joyManualFwd;
          cmdVel.twist.linear.y = maxSpeed / 2.0 * joyManualLeft;
          cmdVel.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;
        }

        pubSpeed->publish(cmdVel);
        pubSkipCount = pubSkipNum;

        if (isRealRobot) {
          if (cmdVel.twist.linear.x == 0.0 && cmdVel.twist.linear.y == 0.0 && cmdVel.twist.angular.z == 0.0) {
            sportReq.StopMove(req);
          } else {
            sportReq.Move(req, cmdVel.twist.linear.x, cmdVel.twist.linear.y, cmdVel.twist.angular.z);
          }
          pubGo2Request->publish(req);
        }
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
